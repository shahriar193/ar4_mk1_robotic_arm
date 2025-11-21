#include <Arduino.h>
#include <math.h>

/* ===================== USER CONSTANTS ===================== */

/* ===== Velocity mode (math-angle deg/s) =====
   Python sends "V v1 v2" where v1,v2 are math-angle velocities in deg/s.
   Teensy continuously steps at those speeds in loop().
*/
static bool  g_vel_mode = false;
static float g_vel_deg_per_sec[2] = {0.0f, 0.0f};
static unsigned long g_vel_next_step_us[2] = {0, 0};



/* PINS (adjust to your wiring) */
static const int J1_PUL = 0;   // STEP
static const int J1_DIR = 1;   // DIR
static const int J1_LIM = 23;  // Limit (NC + INPUT_PULLUP) -> pressed=HIGH

static const int J2_PUL = 4;   // STEP
static const int J2_DIR = 5;   // DIR
static const int J2_LIM = 22;  // Limit (NC + INPUT_PULLUP)

/* STEP signal polarity/timing */
static const bool STEP_ACTIVE_HIGH = false; // false => idle HIGH, pulse LOW
static const int  STEP_PULSE_US    = 20;    // pulse width
static const int  DIR_SETUP_US     = 20;    // DIR setup time before step

/* Motor direction polarity: DIR level that produces CW motion (away from the CCW limit). */
static const bool DIR_CW_LEVEL     = LOW;

/* Homing clear (after contact) — small bump so we’re not riding the switch */
static const long CLEAR_STEPS_J1   = 10;
static const long CLEAR_STEPS_J2   = 10;

/* Debounce */
static const uint16_t DEBOUNCE_MS  = 8;

/* Geometry (mm) — J2 axis relative to J1 axis, and J2 link length */
static const float RX = -63.0f;
static const float RY = -52.0f;
static const float RZ =  65.0f;
static const float L2 = 224.0f;

/* Calibration (steps/deg) */
static const float SPD_J1 = 895.0f;
static const float SPD_J2 = 222.22f;

/* === Angle conventions ===
   J1: math θ1=0° at raw=162.782°. CCW increases math, RAW decreases.
       θ1_math = 162.782 - θ1_raw   ;  θ1_raw = 162.782 - θ1_math
   J2: math θ2 = raw + 18°.  (So when raw=0°, math=+18°.)
       θ2_raw  = θ2_math - 18°
*/
static const float J1_OFFSET_RAW_DEG = 162.782f;
static const float J2_MATH_MINUS_RAW = 18.0f;

/* Motion speed (deg/s) */
static const float MAX_DEG_PER_SEC = 15.0f;

/* ==== SOFT LIMITS (MATH angles, degrees) ==== */
static const float J1_SOFT_MIN_MATH = -120.0f;
static const float J1_SOFT_MAX_MATH = +165.0f;  // your new upper soft limit
static const float J1_LIM_EPS       = 0.0f;     // safety margin; set to ~0.5 if desired

static const float J2_SOFT_MIN_MATH = -180.0f;
static const float J2_SOFT_MAX_MATH = +180.0f;

/* ===================== STRUCTS ===================== */
struct JointCfg {
  const char* name;
  int pul, dir, lim;
  bool dir_cw_level;
};
struct JointState {
  long steps = 0;   // +steps = CW direction (away from CCW limit)
  bool homed = false;
};

JointCfg C[2] = {
  { "J1", J1_PUL, J1_DIR, J1_LIM, DIR_CW_LEVEL },
  { "J2", J2_PUL, J2_DIR, J2_LIM, DIR_CW_LEVEL }
};
JointState S[2];

/* Global flag: during homing we ignore soft bands but still respect hard switches. */
static bool g_homing_mode = false;

/* ===================== MATH HELPERS ===================== */
inline bool  feq(float a, float b, float eps=1e-4f){ return fabsf(a-b) < eps; }
inline float clampf(float v, float lo, float hi){ return v < lo ? lo : (v > hi ? hi : v); }

/* Normalize to (-180, 180] */
inline float wrapTo180(float a){
  float r = fmodf(a + 180.0f, 360.0f);
  if (r < 0) r += 360.0f;
  return r - 180.0f;
}

/* Circular distance (deg) between angles a and b (mod 360) */
inline float circ_dist_deg(float a, float b){
  float d = fmodf(a - b, 360.0f);
  if (d >  180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return fabsf(d);
}

/* Choose a J1 target INSIDE [min,max]:
   1) normalize request a -> (-180,180]
   2) candidates {a, a-360, a+360}
   3) keep inside band; pick closest to current (linear |x-curr|)
   4) if none inside, snap to nearest boundary (by circular distance to 'a') */
float select_J1_target_in_band(float a_req, float curr){
  const float lo = J1_SOFT_MIN_MATH + J1_LIM_EPS;
  const float hi = J1_SOFT_MAX_MATH - J1_LIM_EPS;

  float a = wrapTo180(a_req);
  float cands[3] = { a, a - 360.0f, a + 360.0f };

  bool have=false; float best=0.0f, bestDist=1e9f;
  for (int i=0;i<3;i++){
    float x = cands[i];
    if (x >= lo && x <= hi){
      float d = fabsf(x - curr);
      if (d < bestDist){ bestDist=d; best=x; have=true; }
    }
  }
  if (have) return best;

  // no inside candidate -> snap to nearest boundary by circular distance to the *request*
  float d_lo = circ_dist_deg(a, lo);
  float d_hi = circ_dist_deg(a, hi);
  return (d_hi <= d_lo) ? hi : lo;
}

/* ===================== IO HELPERS ===================== */
inline bool limPressed(int i){ return digitalRead(C[i].lim) == HIGH; } // NC+pullup

inline void setDirCW(int i, bool cw){
  digitalWrite(C[i].dir, cw ? C[i].dir_cw_level : !C[i].dir_cw_level);
  delayMicroseconds(DIR_SETUP_US);
}

inline void pulseStepNoGap(int i){
  if (STEP_ACTIVE_HIGH){
    digitalWrite(C[i].pul, HIGH); delayMicroseconds(STEP_PULSE_US);
    digitalWrite(C[i].pul, LOW);
  } else {
    digitalWrite(C[i].pul, LOW);  delayMicroseconds(STEP_PULSE_US);
    digitalWrite(C[i].pul, HIGH);
  }
}

inline void delayDebounce(){ delay(DEBOUNCE_MS); }

/* ===================== ANGLE CONVERSIONS ===================== */
/* raw degrees <-> steps */
inline float stepsToRawDeg_J1(long st){ return st / SPD_J1; }
inline float stepsToRawDeg_J2(long st){ return st / SPD_J2; }
inline long  rawDegToSteps_J1(float deg){ return lroundf(deg * SPD_J1); }
inline long  rawDegToSteps_J2(float deg){ return lroundf(deg * SPD_J2); }

/* math <-> raw (conventions) */
inline float mathToRawDeg_J1(float th1_math){ return J1_OFFSET_RAW_DEG - th1_math; }
inline float mathToRawDeg_J2(float th2_math){ return th2_math - J2_MATH_MINUS_RAW; } // raw = math - 18

inline float stepsToMathDeg_J1(long st){
  float raw = stepsToRawDeg_J1(st);
  return J1_OFFSET_RAW_DEG - raw;         // θ1_math = 162.782 - θ1_raw
}
inline float stepsToMathDeg_J2(long st){
  float raw = stepsToRawDeg_J2(st);
  return raw + J2_MATH_MINUS_RAW;         // θ2_math = θ2_raw + 18
}

/* ===================== PER-STEP GUARDS ===================== */
/* Return true if a J1 step in 'dirCW' should be blocked by the soft band.
   During homing we ignore soft band, but still respect hard switches elsewhere. */
bool j1_soft_guard_block(bool dirCW){
  if (g_homing_mode) return false;  // allow soft-band violations during homing moves

  const float lo = J1_SOFT_MIN_MATH + J1_LIM_EPS;
  const float hi = J1_SOFT_MAX_MATH - J1_LIM_EPS;
  const float dth = 1.0f / SPD_J1;              // math change per step
  float curr = stepsToMathDeg_J1(S[0].steps);

  // If currently above/ below the band, only allow the step that moves us toward re-entry.
  if (curr > hi) return !dirCW;   // above band → allow CW (math decreases), block CCW
  if (curr < lo) return  dirCW;   // below band → allow CCW (math increases), block CW

  // Inside the band: block if next step would leave the band.
  float next = dirCW ? (curr - dth) : (curr + dth);
  return (next < lo) || (next > hi);
}

/* One protected step:
   - Blocks motion toward a PRESSED hard limit (for both joints).
   - Enforces J1 soft band step-by-step (unless g_homing_mode).
*/
bool tryStep(int i, bool dirCW){
  // Hard limit protection: if the limit switch is pressed, you may only move AWAY from it.
  if (limPressed(i)){
    // Limit is on the CCW side for both joints. Moving CCW (dirCW==false) is "toward" it.
    if (!dirCW) return false;     // block further CCW while pressed
  }

  // J1 soft band guard
  if (i==0 && j1_soft_guard_block(dirCW)) return false;

  setDirCW(i, dirCW);
  pulseStepNoGap(i);
  S[i].steps += dirCW ? +1 : -1;  // +steps = CW, -steps = CCW
  return true;
}

// The below is added for velocity control
void vel_update(){
  if (!g_vel_mode) return;

  unsigned long now = micros();

  for (int i = 0; i < 2; ++i){
    float vel = g_vel_deg_per_sec[i];   // math-angle deg/s

    // Tiny velocity => ignore
    if (fabsf(vel) < 1e-3f) continue;

    // Steps/s = |vel_deg_per_s| * steps_per_deg
    float spd = (i == 0 ? SPD_J1 : SPD_J2);
    float steps_per_sec = fabsf(vel) * spd;
    if (steps_per_sec < 1e-3f) continue;

    unsigned long period = (unsigned long)max(2.0f, 1e6f / steps_per_sec);

    // Time to step?
    if ((long)(now - g_vel_next_step_us[i]) >= 0){
      // Map math-angle sign -> dirCW:
      //   J1: dirCW step --> math DECREASES  (see j1_soft_guard)
      //   J2: dirCW step --> math INCREASES
      bool dirCW;
      if (i == 0){
        // J1: vel>0 (math++) => dirCW must be false (CCW step)
        dirCW = (vel < 0.0f);
      } else {
        // J2: vel>0 (math++) => dirCW must be true  (CW step)
        dirCW = (vel > 0.0f);
      }

      if (tryStep(i, dirCW)){
        g_vel_next_step_us[i] = now + period;
      } else {
        // Blocked by limit / soft band -> zero this joint’s velocity
        g_vel_deg_per_sec[i] = 0.0f;
        Serial.print("[VEL] Joint ");
        Serial.print(i);
        Serial.println(" blocked; velocity set to 0.");
      }
    }
  }

  // If both velocities are ~0, drop out of vel mode
  if (fabsf(g_vel_deg_per_sec[0]) < 1e-3f &&
      fabsf(g_vel_deg_per_sec[1]) < 1e-3f){
    g_vel_mode = false;
  }
}


/* ===================== FK (math angles in degrees) ===================== */
/* p = Rz(theta1) * ( [RX, RY, RZ] + [L2*cos(theta2), 0, L2*sin(theta2)] ) */
void fk_mm(float th1_math_deg, float th2_math_deg, float &x, float &y, float &z){
  float t1 = th1_math_deg * (PI/180.0f);
  float t2 = th2_math_deg * (PI/180.0f);
  float ca = cosf(t1), sa = sinf(t1);
  float c2 = cosf(t2), s2 = sinf(t2);

  float A  = RX + L2 * c2;  // x before yaw
  float B  = RY;            // y before yaw
  x = ca*A - sa*B;
  y = sa*A + ca*B;
  z = RZ + L2 * s2;
}

/* ===================== IK with clamping ===================== */
void ik_solve_deg_clamped(float xd, float yd, float zd,
                          float &th1_math_deg, float &th2_math_deg,
                          float &x_closest, float &y_closest, float &z_closest)
{
  // clamp z
  float zmin = RZ - L2, zmax = RZ + L2;
  float zc = min(max(zd, zmin), zmax);

  // theta2 from clamped z
  float s2 = (zc - RZ) / L2;
  s2 = max(-1.0f, min(1.0f, s2));
  float t2 = asinf(s2);
  float c2 = cosf(t2);

  // compute theta1 keeping azimuth
  float A  = RX + L2 * c2;            // x-before-yaw
  float psi = atan2f(yd, xd);         // desired azimuth
  float phi = atan2f(RY, A);          // shoulder offset angle
  float t1  = psi - phi;

  th1_math_deg = t1 * 180.0f/PI;
  th2_math_deg = t2 * 180.0f/PI;

  // closest point (same psi, fixed rho, zc)
  float rho = sqrtf(A*A + RY*RY);
  x_closest = rho * cosf(psi);
  y_closest = rho * sinf(psi);
  z_closest = zc;
}

/* ===================== HOMING ===================== */
/* Rules:
   - J1: drive CCW to limit; zero steps at contact; clear 10 steps; then go to raw=162.782° (math 0°).
         During homing, soft band is ignored; hard switch is enforced.
   - J2: drive CCW to limit; clear 10 steps; set steps=0 there (raw=0° → math=+18°).
         After homing, if switch is pressed at any time, we won't allow CCW until cleared.
*/
bool home_one_J1(){

  g_vel_mode = false;
  g_vel_deg_per_sec[0] = 0.0f;
  g_vel_deg_per_sec[1] = 0.0f;

  Serial.println("[J1] Homing...");
  g_homing_mode = true; // ignore soft band during homing path

  // CCW until pressed
  if (!limPressed(0)){
    unsigned long t0 = millis();
    setDirCW(0, /*CW?*/false);
    while (!limPressed(0)){
      pulseStepNoGap(0);
      S[0].steps -= 1; // CCW
      if (millis() - t0 > 120000UL){ Serial.println("  timeout before contact"); g_homing_mode=false; return false; }
      delayMicroseconds(200);
    }
  }
  delayDebounce();

  // Zero steps at contact (raw=0 at limit)
  S[0].steps = 0;

  // Clear a little CW to avoid riding the switch
  setDirCW(0, true);
  for (long k=0; k<CLEAR_STEPS_J1; ++k){ pulseStepNoGap(0); S[0].steps += 1; delayMicroseconds(200); }
  delayDebounce();

  // Go to raw = 162.782 deg (math zero). Soft band ignored during this move.
  long target = rawDegToSteps_J1(J1_OFFSET_RAW_DEG);
  while (S[0].steps != target){
    bool cw = (target > S[0].steps);
    // Even in homing, respect hard switch: if pressed, disallow CCW.
    if (!tryStep(0, cw)){ Serial.println("  blocked while moving to J1 math-zero."); g_homing_mode=false; return false; }
    delayMicroseconds(200);
  }

  g_homing_mode = false; // restore normal guards
  S[0].homed = true;
  Serial.print("[J1] Homed at raw="); Serial.print(stepsToRawDeg_J1(S[0].steps),3);
  Serial.println(" deg (math θ1 = 0)");
  return true;
}

bool home_one_J2(){
  g_vel_mode = false;
  g_vel_deg_per_sec[0] = 0.0f;
  g_vel_deg_per_sec[1] = 0.0f;

  Serial.println("[J2] Homing...");

  // CCW until pressed
  if (!limPressed(1)){
    unsigned long t0 = millis();
    setDirCW(1, /*CW?*/false);
    while (!limPressed(1)){
      pulseStepNoGap(1);
      S[1].steps -= 1; // CCW
      if (millis() - t0 > 120000UL){ Serial.println("  timeout before contact"); return false; }
      delayMicroseconds(200);
    }
  }
  delayDebounce();

  // Clear CW 10 steps and define that as raw=0°
  setDirCW(1, true);
  for (long k=0; k<CLEAR_STEPS_J2; ++k){ pulseStepNoGap(1); S[1].steps += 1; delayMicroseconds(200); }
  delayDebounce();

  S[1].steps = 0; // raw=0° here (math=+18°)
  S[1].homed = true;
  Serial.println("[J2] Homed at raw=0° (math θ2 = +18°).");
  return true;
}

bool home_all(){
  g_vel_mode = false;
  g_vel_deg_per_sec[0] = 0.0f;
  g_vel_deg_per_sec[1] = 0.0f;

  Serial.println("=== HOMING START ===");
  if (!home_one_J1()) return false;
  if (!home_one_J2()) return false;
  Serial.println("=== HOMING DONE ===");
  return true;
}

/* ===================== COORDINATED MOVE (MAX_DEG_PER_SEC) ===================== */
bool move_to_steps_coordinated(long tgtJ1, long tgtJ2){

  g_vel_mode = false;
  g_vel_deg_per_sec[0] = 0.0f;
  g_vel_deg_per_sec[1] = 0.0f;

  long rem1 = tgtJ1 - S[0].steps;
  long rem2 = tgtJ2 - S[1].steps;
  if (rem1==0 && rem2==0) return true;

  float rate1 = SPD_J1 * MAX_DEG_PER_SEC; // steps/s
  float rate2 = SPD_J2 * MAX_DEG_PER_SEC; // steps/s
  unsigned long period1 = (rem1!=0) ? (unsigned long)max(2.0f, 1e6f / rate1) : 0;
  unsigned long period2 = (rem2!=0) ? (unsigned long)max(2.0f, 1e6f / rate2) : 0;

  bool dir1 = (rem1 >= 0); // CW if positive remaining
  bool dir2 = (rem2 >= 0);

  unsigned long next1 = micros();
  unsigned long next2 = micros();

  while (rem1!=0 || rem2!=0){
    unsigned long now = micros();

    if (rem1!=0 && (long)(now - next1) >= 0){
      if (!tryStep(0, dir1)) return false;
      next1 += period1;
      rem1 += dir1 ? -1 : +1;
    }
    if (rem2!=0 && (long)(now - next2) >= 0){
      if (!tryStep(1, dir2)) return false;
      next2 += period2;
      rem2 += dir2 ? -1 : +1;
    }
  }
  return true;
}

/* ===================== ANGLE-TO-TARGET MOVES ===================== */
bool move_to_math_deg(float th1_math_req, float th2_math_req){

  g_vel_mode = false;
  g_vel_deg_per_sec[0] = 0.0f;
  g_vel_deg_per_sec[1] = 0.0f;

  // J1: select an in-band target nearest to current
  float curr = stepsToMathDeg_J1(S[0].steps);
  float th1_cmd = select_J1_target_in_band(th1_math_req, curr);

  // J2: clamp to soft window
  float th2_cmd = clampf(th2_math_req, J2_SOFT_MIN_MATH, J2_SOFT_MAX_MATH);

  // Convert to raw targets
  float th1_raw = mathToRawDeg_J1(th1_cmd);
  float th2_raw = mathToRawDeg_J2(th2_cmd);

  long tJ1 = rawDegToSteps_J1(th1_raw);
  long tJ2 = rawDegToSteps_J2(th2_raw);

  Serial.print("[TARGET:MATH] J1 req="); Serial.print(th1_math_req,3);
  Serial.print(" -> cmd=");              Serial.print(th1_cmd,3);
  Serial.print("°,   J2 req=");          Serial.print(th2_math_req,3);
  Serial.print(" -> cmd=");              Serial.print(th2_cmd,3); Serial.println("°");

  bool ok = move_to_steps_coordinated(tJ1, tJ2);
  if (ok) print_status();
  else    Serial.println("[MOVE:MATH] blocked/failed.");
  return ok;
}

bool move_to_raw_deg(float th1_raw_req, float th2_raw_req){
  g_vel_mode = false;
  g_vel_deg_per_sec[0] = 0.0f;
  g_vel_deg_per_sec[1] = 0.0f;

  // Convert raw→math first so J1 uses the same constrained selection
  float th1_m_req = J1_OFFSET_RAW_DEG - th1_raw_req;      // J1 math
  float th2_m_req = th2_raw_req + J2_MATH_MINUS_RAW;      // J2 math
  return move_to_math_deg(th1_m_req, th2_m_req);
}

/* ===================== IK ANALYZER (reachability & nearest point) ===================== */
struct IKReport {
  float th1_m_req, th2_m_req;   // raw IK request (math)
  float th1_m_cmd, th2_m_cmd;   // after J1 constrained selection & J2 clamp
  float th1_raw,   th2_raw;
  long  tJ1, tJ2;
  float xC, yC, zC;
  float dx, dy, dz;
  float dist;
  bool  clampedZ;
  bool  clampedXY;
  bool  j1Snapped;
};

bool analyze_xyz(float xd, float yd, float zd, bool also_move){
  IKReport R{};
  float xC, yC, zC;

  // IK (geom clamp)
  ik_solve_deg_clamped(xd, yd, zd, R.th1_m_req, R.th2_m_req, xC, yC, zC);

  // J1 constrained selection
  float curr = stepsToMathDeg_J1(S[0].steps);
  R.th1_m_cmd = select_J1_target_in_band(R.th1_m_req, curr);
  R.j1Snapped = !feq(R.th1_m_req, R.th1_m_cmd);

  // J2 clamp
  R.th2_m_cmd = clampf(R.th2_m_req, J2_SOFT_MIN_MATH, J2_SOFT_MAX_MATH);

  // detect IK clamping
  R.clampedZ = !feq(zC, zd);
  float t2 = R.th2_m_cmd * (PI/180.0f);
  float rho = sqrtf( (RX + L2*cosf(t2))*(RX + L2*cosf(t2)) + RY*RY );
  float rd  = hypotf(xd, yd);
  R.clampedXY = !feq(rd, rho);

  // Convert to raw & steps
  R.th1_raw = mathToRawDeg_J1(R.th1_m_cmd);
  R.th2_raw = mathToRawDeg_J2(R.th2_m_cmd);
  R.tJ1 = rawDegToSteps_J1(R.th1_raw);
  R.tJ2 = rawDegToSteps_J2(R.th2_raw);

  // Closest point and errors
  R.xC = xC; R.yC = yC; R.zC = zC;
  R.dx = xd - xC; R.dy = yd - yC; R.dz = zd - zC;
  
  // OOPS: fix the line above:
  R.dist = sqrtf(R.dx*R.dx + R.dy*R.dy + R.dz*R.dz);

  // Print
  Serial.println("=== IK ANALYSIS ===");
  Serial.print("Desired:     x="); Serial.print(xd,3);
  Serial.print("  y=");           Serial.print(yd,3);
  Serial.print("  z=");           Serial.println(zd,3);

  Serial.print("Closest:     x="); Serial.print(R.xC,3);
  Serial.print("  y=");           Serial.print(R.yC,3);
  Serial.print("  z=");           Serial.println(R.zC,3);

  Serial.print("Error:       dx="); Serial.print(R.dx,3);
  Serial.print("  dy=");           Serial.print(R.dy,3);
  Serial.print("  dz=");           Serial.print(R.dz,3);
  Serial.print("  |e|=");         Serial.println(R.dist,3);

  Serial.print("Angles MATH (req→cmd): θ1="); Serial.print(R.th1_m_req,3);
  Serial.print("→");                           Serial.print(R.th1_m_cmd,3);
  Serial.print("°,  θ2=");                     Serial.print(R.th2_m_req,3);
  Serial.print("→");                           Serial.print(R.th2_m_cmd,3); Serial.println("°");

  Serial.print("Angles RAW:  θ1="); Serial.print(R.th1_raw,3);
  Serial.print("°, θ2=");          Serial.print(R.th2_raw,3); Serial.println("°");

  Serial.print("Steps:       J1="); Serial.print(R.tJ1);
  Serial.print("  J2=");           Serial.println(R.tJ2);

  Serial.print("Clamping:    Z="); Serial.print(R.clampedZ ? "YES" : "no");
  Serial.print("  XY=");           Serial.print(R.clampedXY ? "YES" : "no");
  Serial.print("  J1 snapped=");  Serial.println(R.j1Snapped ? "YES" : "no");

  if (also_move){
    bool ok = move_to_steps_coordinated(R.tJ1, R.tJ2);
    if (!ok){ Serial.println("[MOVE] blocked."); return false; }
    print_status();
  }
  return true;
}

/* ===================== HIGH-LEVEL: MOVE TO CARTESIAN ===================== */
bool move_to_xyz_mm(float xd, float yd, float zd){
  float th1_m_req, th2_m_req, xC, yC, zC;
  ik_solve_deg_clamped(xd, yd, zd, th1_m_req, th2_m_req, xC, yC, zC);

  float curr = stepsToMathDeg_J1(S[0].steps);
  float th1_m_cmd = select_J1_target_in_band(th1_m_req, curr);
  float th2_m_cmd = clampf(th2_m_req, J2_SOFT_MIN_MATH, J2_SOFT_MAX_MATH);

  // IK clamping flags for print
  bool clampedZ  = !feq(zC, zd);
  float t2 = th2_m_cmd * (PI/180.0f);
  float rho = sqrtf( (RX + L2*cosf(t2))*(RX + L2*cosf(t2)) + RY*RY );
  bool clampedXY = !feq(hypotf(xd,yd), rho);
  bool j1Snapped = !feq(th1_m_cmd, th1_m_req);

  // Convert to raw & move
  long tJ1 = rawDegToSteps_J1( mathToRawDeg_J1(th1_m_cmd) );
  long tJ2 = rawDegToSteps_J2( mathToRawDeg_J2(th2_m_cmd) );

  Serial.print("[TARGET:XYZ] x="); Serial.print(xd,2);
  Serial.print(" y=");             Serial.print(yd,2);
  Serial.print(" z=");             Serial.print(zd,2);
  Serial.print(" -> MATH θ1 req→cmd "); Serial.print(th1_m_req,3); Serial.print("→"); Serial.print(th1_m_cmd,3);
  Serial.print("°, θ2 req→cmd ");       Serial.print(th2_m_req,3); Serial.print("→"); Serial.print(th2_m_cmd,3);
  Serial.print("°  (IK Z=");            Serial.print(clampedZ ? "clamped" : "ok");
  Serial.print(", XY=");                Serial.print(clampedXY ? "clamped" : "ok");
  Serial.print(", J1-snap=");           Serial.print(j1Snapped ? "YES" : "no"); Serial.println(")");

  Serial.print("Closest IK point: x="); Serial.print(xC,2);
  Serial.print(" y=");                  Serial.print(yC,2);
  Serial.print(" z=");                  Serial.println(zC,2);

  bool ok = move_to_steps_coordinated(tJ1, tJ2);
  if (!ok) Serial.println("[MOVE] blocked (limit/e-stop).");
  else     print_status();
  return ok;
}

/* ===================== PRINT STATUS ===================== */
void print_status(){
  long s1 = S[0].steps, s2 = S[1].steps;

  float r1 = stepsToRawDeg_J1(s1);
  float r2 = stepsToRawDeg_J2(s2);

  float m1 = stepsToMathDeg_J1(s1);
  float m2 = stepsToMathDeg_J2(s2);

  float x,y,z; fk_mm(m1,m2,x,y,z);

  Serial.print("[J1] steps="); Serial.print(s1);
  Serial.print("  raw_deg=");   Serial.print(r1,3);
  Serial.print("  math_deg=");  Serial.println(m1,3);

  Serial.print("[J2] steps="); Serial.print(s2);
  Serial.print("  raw_deg=");   Serial.print(r2,3);
  Serial.print("  math_deg=");  Serial.println(m2,3);

  Serial.print("[EE] x="); Serial.print(x,2);
  Serial.print(" mm  y="); Serial.print(y,2);
  Serial.print(" mm  z="); Serial.print(z,2);
  Serial.println(" mm");
}

/* ===================== SERIAL UI ===================== */
/* Commands:
   h                 -> home all (J1 → math 0 @ raw=162.782°, J2 home raw=0°→math=+18°)
   p                 -> print status (angles, steps, FK)
   Q x y z           -> analyze reachability; print nearest point (NO MOVE)
   G x y z           -> go to Cartesian (mm), IK with clamping, 15 deg/s
   M th1 th2         -> go to math (deg) — J1 constrained inside [-120, +165]
   R th1 th2         -> go to raw (deg) via math conversion then J1 constrained
   a/d (with 1/2)    -> manual jog CCW/CW (J1 jog obeys soft limit; hard switch direction locked)
   1 / 2             -> select joint for jog
   s / SPACE         -> stop jogging
*/
int sel = 0; // 0=J1, 1=J2
enum Jog {JOG_NONE=0, JOG_CW=+1, JOG_CCW=-1} jog = JOG_NONE;
bool e_stop = false;

void handle_serial(){
  static char buf[96];
  static int len=0;

  while (Serial.available()){
    int c = Serial.read();
    if (c=='\r' || c=='\n'){
      if (len>0){
        buf[len]=0;

        if (buf[0]=='Q' || buf[0]=='q'){
          float x,y,z;
          if (sscanf(buf+1, "%f %f %f", &x,&y,&z)==3){
            analyze_xyz(x,y,z, /*also_move=*/false);
          } else {
            Serial.println("Usage: Q x y z   (mm) — analyze only; no motion");
          }
        }
        else if (buf[0]=='G' || buf[0]=='g'){
          float x,y,z;
          if (sscanf(buf+1, "%f %f %f", &x,&y,&z)==3){
            e_stop=false;
            move_to_xyz_mm(x,y,z);
          } else {
            Serial.println("Usage: G x y z   (mm)");
          }
        }
        else if (buf[0]=='M' || buf[0]=='m'){
          float a1, a2;
          if (sscanf(buf+1, "%f %f", &a1, &a2)==2){
            e_stop=false;
            move_to_math_deg(a1, a2);
          } else {
            Serial.println("Usage: M th1_deg th2_deg  (math/trig angles)");
          }
        }
        else if (buf[0]=='R' || buf[0]=='r'){
          float a1, a2;
          if (sscanf(buf+1, "%f %f", &a1, &a2)==2){
            e_stop=false;
            move_to_raw_deg(a1, a2);
          } else {
            Serial.println("Usage: R th1_deg th2_deg  (raw angles)");
          }
        }
        else if (buf[0]=='V' || buf[0]=='v'){
          float v1, v2;
          if (sscanf(buf+1, "%f %f", &v1, &v2)==2){
            // Clamp to firmware max speed for safety
            float vmax = MAX_DEG_PER_SEC;
            if (v1 >  vmax) v1 = vmax;
            if (v1 < -vmax) v1 = -vmax;
            if (v2 >  vmax) v2 = vmax;
            if (v2 < -vmax) v2 = -vmax;

            g_vel_deg_per_sec[0] = v1;
            g_vel_deg_per_sec[1] = v2;

            // Enable velocity mode if any non-zero vel
            g_vel_mode = (fabsf(v1) > 1e-3f) || (fabsf(v2) > 1e-3f);

            // Reset step timers so we respond promptly
            unsigned long now = micros();
            g_vel_next_step_us[0] = now;
            g_vel_next_step_us[1] = now;

            Serial.print("[VEL] J1=");
            Serial.print(v1, 3);
            Serial.print(" deg/s, J2=");
            Serial.print(v2, 3);
            Serial.println(" deg/s");
          } else {
            Serial.println("Usage: V v1_deg_per_s v2_deg_per_s  (math-angle velocities)");
          }
        }



        else {
          char k = buf[0];
          if (k=='h'||k=='H'){ e_stop=false; S[0].homed=S[1].homed=false; home_all(); }
          else if (k=='p'||k=='P'){ print_status(); }
          else if (k=='1'){ sel=0; Serial.println("Selected J1"); }
          else if (k=='2'){ sel=1; Serial.println("Selected J2"); }
          else if (k=='a'||k=='A'){ jog=JOG_CCW; }
          else if (k=='d'||k=='D'){ jog=JOG_CW; }
          else if (k=='s'||k=='S'||k==' '){ jog=JOG_NONE; Serial.println("STOP"); }
          else if (k=='e'||k=='E'){ e_stop=false; Serial.println("E-STOP cleared"); }
          else {
            Serial.println("Cmds: h | p | Q x y z | G x y z | M th1 th2 | R th1 th2 | 1/2 sel | a CCW | d CW | s stop");
          }
        }
        len=0;
      }
    } else {
      if (len<95) buf[len++]= (char)c;
    }
  }
}

/* ===================== Arduino lifecycle ===================== */
void setup(){
  Serial.begin(115200);
  delay(150);
  Serial.println("2-DOF (J1 yaw + J2 pitch) — J1 soft-limit [-120,+165] enforced after homing; hard switch direction-locked; FK/IK");
  Serial.println("Cmds: h | p | Q x y z | G x y z | M th1 th2 | R th1 th2 | 1/2 sel | a CCW | d CW | s stop");

  pinMode(J1_PUL, OUTPUT); pinMode(J1_DIR, OUTPUT); pinMode(J1_LIM, INPUT_PULLUP);
  pinMode(J2_PUL, OUTPUT); pinMode(J2_DIR, OUTPUT); pinMode(J2_LIM, INPUT_PULLUP);

  digitalWrite(J1_PUL, STEP_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(J2_PUL, STEP_ACTIVE_HIGH ? LOW : HIGH);

  e_stop=false;
  if (!home_all()){
    Serial.println("[BOOT] Homing FAILED; fix wiring/limits and press 'h' to retry.");
  } else {
    print_status();
  }
}

void loop(){
  handle_serial();

  if (e_stop) {
    jog = JOG_NONE;
    // Also kill velocities
    g_vel_mode = false;
    g_vel_deg_per_sec[0] = 0.0f;
    g_vel_deg_per_sec[1] = 0.0f;
    return;
  }

  // Continuous velocity control (non-blocking)
  vel_update();

  // Manual jog only when NOT in velocity mode
  if (!g_vel_mode && jog != JOG_NONE){
    int i = sel;
    bool dirCW = (jog == JOG_CW);

    // Don't push further into a pressed hardware limit
    if (limPressed(i) && !dirCW) return;

    unsigned long period = (unsigned long)max(2.0f,
      1e6f / ((i==0?SPD_J1:SPD_J2) * MAX_DEG_PER_SEC));

    static unsigned long next_due_us[2] = {0,0};
    unsigned long now = micros();
    if ((long)(now - next_due_us[i]) >= 0){
      if (tryStep(i, dirCW)){
        next_due_us[i] = now + period;
      }
    }
  }
}

