#include <Arduino.h>
#include <limits.h>
#include <math.h>

/* ================== FORWARD DECLARATIONS ================== */
static inline long round_to_long(float x);
inline bool rawPressed(int i);
bool stablePressed(int i);
void primeDebouncers(unsigned long settle_ms = 20);
inline bool awayIsCW(int i);
inline void setDirCW(int i, bool cw);
inline void pulseStep(int i);
inline void pulseStepNoGap(int i);
bool stepOnce(int i, bool dirCW);
bool tryStepNoGap(int i, bool dirCW);
float stepsToDeg(int i);
long  degToSteps(int i, float deg);
bool moveStepsRel(int i, long dSteps);
bool homeOne(int i);
bool moveAllToStepsSimul(const long target_in[2]);
bool moveAllToAbsDegreesSimul(const float degs[2]);
bool initialize_all();
void printStatus(int i);
void printAllLimits();
void handleSerial();

/* ================== USER CONFIG (Angles & Calibration) ==================
   Convention:
     - Zero (0 steps) is at the **CCW limit switch** after homing.
     - Positive steps/angles are **away from CCW limit** (CW direction).
*/
static const float THEO_STEPS_PER_DEG_J1 = 694.444f;  // ≈ (1600 * 156.25) / 360
static const float THEO_STEPS_PER_DEG_J2 = 222.222f;  // ≈ (1600 * 50)     / 360

// Optional tiny trims so console angles match your marks exactly:
static const float CAL_TRIM_J1 = 1.00000f;  // e.g., 694.31/694.44 ≈ 0.99981
static const float CAL_TRIM_J2 = 1.00000f;  // e.g., 223.45/222.22 ≈ 1.00553

static const float STEPS_PER_DEG_J1 = THEO_STEPS_PER_DEG_J1 * CAL_TRIM_J1;
static const float STEPS_PER_DEG_J2 = THEO_STEPS_PER_DEG_J2 * CAL_TRIM_J2;

/* ================== Pins & Electrical ==================
   Teensy 4.1. Limits: NC->GND, COM->pin (INPUT_PULLUP). Pressed = HIGH.
*/
static const int J1_PUL = 0;    // step
static const int J1_DIR = 1;    // dir
static const int J1_LIM = 23;   // limit (NC+pullup -> pressed=HIGH)

static const int J2_PUL = 4;    // step
static const int J2_DIR = 5;    // dir
static const int J2_LIM = 22;   // limit

// Electrical & timing
static const bool STEP_ACTIVE_HIGH = false;  // false: idle HIGH, pulse LOW
static const int  STEP_PULSE_US    = 20;
static const int  STEP_GAP_US_J1   = 100;    // speed J1
static const int  STEP_GAP_US_J2   = 350;    // speed J2
static const bool DIR_CW_LEVEL     = LOW;    // flip if your wiring reverses
static const uint16_t DEBOUNCE_MS  = 10;
static const int  DIR_SETUP_US     = 20;

/* ================== Global target angles (deg from CCW limit) ================== */
static const float J1_HOME_DEG  = 180.0f;  // Home target for J1
static const float J2_HOME_DEG  = 90.0f;   // Home target for J2
static const float J1_POSE2_DEG = 90t.0f;   // Pose2 target for J1
static const float J2_POSE2_DEG = 45.0f;   // Pose2 target for J2

static const long  J1_CLEAR_STEPS = 100;   // back-off after release
static const long  J2_CLEAR_STEPS = 100;

/* ================== Types ================== */
struct JointCfg {
  const char* name;
  int pul, dir, lim;
  bool has_limit;
  bool limit_at_CW;      // false => limit is at CCW end
  bool dir_cw_level;     // DIR level that yields CW at motor
  int  step_gap_us;
  int  step_pulse_us;
  bool pulse_active_high;
  float steps_per_deg;
  long soft_min, soft_max;
  long clear_extra;
};

struct JointState {
  long steps = 0;          // + away from CCW limit (CW), 0 at switch
  bool lastStable = false; // debounced pressed state
  bool lastRaw    = false; // instantaneous
  unsigned long tChange = 0;
  bool homed = false;
};

/* ================== Soft limits (converted from degrees you asked) ================== */
static const long J1_SOFT_MIN_STEPS = 0;
static const long J1_SOFT_MAX_STEPS = (long)(STEPS_PER_DEG_J1 * 300.0f + 0.5f);  // 300°
static const long J2_SOFT_MIN_STEPS = 0;
static const long J2_SOFT_MAX_STEPS = (long)(STEPS_PER_DEG_J2 * 250.0f + 0.5f);  // 250°

/* ================== Config (2 joints) ==================
   Both joints: limit at **CCW** end → limit_at_CW = false.
*/
JointCfg C[2] = {
  // name, PUL,DIR,LIM, hasLim, lim@CW, dirCW,         gap,             pulse,        actHi,           steps/deg,           softMin,               softMax,               clear
  { "J1", J1_PUL,J1_DIR,J1_LIM, true,   false, DIR_CW_LEVEL, STEP_GAP_US_J1, STEP_PULSE_US, STEP_ACTIVE_HIGH, STEPS_PER_DEG_J1, J1_SOFT_MIN_STEPS,    J1_SOFT_MAX_STEPS,    J1_CLEAR_STEPS },
  { "J2", J2_PUL,J2_DIR,J2_LIM, true,   false, DIR_CW_LEVEL, STEP_GAP_US_J2, STEP_PULSE_US, STEP_ACTIVE_HIGH, STEPS_PER_DEG_J2, J2_SOFT_MIN_STEPS,    J2_SOFT_MAX_STEPS,    J2_CLEAR_STEPS }
};

JointState S[2];

// Saved "home pose" (post-initialize), for 'u'
long saved_home_steps[2] = {0,0};
bool home_pose_saved = false;

/* ================== Keyboard control ================== */
int sel = 0; // 0=J1, 1=J2
enum Jog { JOG_NONE=0, JOG_CW=+1, JOG_CCW=-1 } jog_cmd = JOG_NONE;
volatile bool e_stop = false;

/* ================== Helpers ================== */
static inline long round_to_long(float x){ return (x >= 0.0f) ? (long)(x + 0.5f) : (long)(x - 0.5f); }

inline bool rawPressed(int i){ return C[i].has_limit ? (digitalRead(C[i].lim) == HIGH) : false; } // NC+PULLUP

bool stablePressed(int i){
  if (!C[i].has_limit) return false;
  bool r = rawPressed(i);
  if (r != S[i].lastRaw){ S[i].lastRaw = r; S[i].tChange = millis(); }
  if ((unsigned long)(millis() - S[i].tChange) >= DEBOUNCE_MS) S[i].lastStable = r;
  return S[i].lastStable;
}

void primeDebouncers(unsigned long settle_ms){
  delay(settle_ms);
  for (int i=0;i<2;++i){
    bool r = rawPressed(i);
    S[i].lastRaw = S[i].lastStable = r;
    S[i].tChange = millis();
  }
}

// limit at CCW → AWAY is CW
inline bool awayIsCW(int i){ return !C[i].limit_at_CW; }

inline void setDirCW(int i, bool cw){
  digitalWrite(C[i].dir, cw ? C[i].dir_cw_level : !C[i].dir_cw_level);
  delayMicroseconds(DIR_SETUP_US);
}

inline void pulseStep(int i){
  if (C[i].pulse_active_high){ digitalWrite(C[i].pul, HIGH); delayMicroseconds(C[i].step_pulse_us); digitalWrite(C[i].pul, LOW); }
  else {                        digitalWrite(C[i].pul, LOW);  delayMicroseconds(C[i].step_pulse_us); digitalWrite(C[i].pul, HIGH); }
  delayMicroseconds(C[i].step_gap_us);
}

inline void pulseStepNoGap(int i){
  if (C[i].pulse_active_high){ digitalWrite(C[i].pul, HIGH); delayMicroseconds(C[i].step_pulse_us); digitalWrite(C[i].pul, LOW); }
  else {                        digitalWrite(C[i].pul, LOW);  delayMicroseconds(C[i].step_pulse_us); digitalWrite(C[i].pul, HIGH); }
}

// One protected microstep; returns false if blocked (limit toward or soft cap or E-STOP)
bool stepOnce(int i, bool dirCW){
  if (e_stop) return false;

  const bool away = (dirCW == awayIsCW(i)); // with our config, away == dirCW (true)

  // Limit guard: if switch is pressed, block TOWARD, allow AWAY to clear
  if (C[i].has_limit && stablePressed(i) && !away) return false;

  // Software limits (relative to zero at limit)
  long next = S[i].steps + (away ? +1 : -1);
  if (next < C[i].soft_min || next > C[i].soft_max) return false;

  setDirCW(i, dirCW);
  pulseStep(i);

  // Update position
  S[i].steps = next;
  return true;
}

// Same but without trailing per-joint gap (for coordinated stepping)
bool tryStepNoGap(int i, bool dirCW){
  if (e_stop) return false;

  const bool away = (dirCW == awayIsCW(i));
  if (C[i].has_limit && stablePressed(i) && !away) return false;

  long next = S[i].steps + (away ? +1 : -1);
  if (next < C[i].soft_min || next > C[i].soft_max) return false;

  setDirCW(i, dirCW);
  pulseStepNoGap(i);
  S[i].steps = next;
  return true;
}

float stepsToDeg(int i){ return S[i].steps / C[i].steps_per_deg; }
long  degToSteps(int i, float deg){ return round_to_long(deg * C[i].steps_per_deg); }

bool moveStepsRel(int i, long dSteps){
  if (dSteps == 0) return true;
  bool dirCW = (dSteps > 0) ? awayIsCW(i) : !awayIsCW(i);
  long remain = labs(dSteps);
  while (remain-- > 0){
    if (!stepOnce(i, dirCW)) return false;
  }
  return true;
}

/* ================== Homing (CCW to switch → zero → CW clear) ================== */
bool homeOne(int i){
  Serial.print("["); Serial.print(C[i].name); Serial.println("] Homing toward CCW limit");

  // If not pressed, move CCW (toward limit) until pressed
  if (!stablePressed(i)){
    const bool towardCW = false; // homing toward CCW
    unsigned long t0 = millis();
    while (!stablePressed(i)){
      // Temporarily relax soft limits until first contact
      if (!stepOnce(i, towardCW)){
        long mn=C[i].soft_min, mx=C[i].soft_max;
        C[i].soft_min = LONG_MIN/2; C[i].soft_max = LONG_MAX/2;
        bool ok = stepOnce(i, towardCW);
        C[i].soft_min = mn; C[i].soft_max = mx;
        if (!ok){ Serial.println("  blocked before contact."); return false; }
      }
      if ((unsigned long)(millis() - t0) > 120000UL){ Serial.println("  timeout to contact."); return false; }
    }
  } else {
    Serial.println("  already on switch.");
  }

  // Zero at contact, then back off AWAY (CW) until release + extra clearance
  S[i].steps = 0;
  unsigned long t1 = millis();
  while (stablePressed(i)){
    if (!stepOnce(i, /*dirCW=*/true)){ Serial.println("  back-off blocked."); return false; }
    if ((unsigned long)(millis() - t1) > 120000UL){ Serial.println("  back-off timeout."); return false; }
  }
  if (!moveStepsRel(i, C[i].clear_extra)) return false;

  S[i].homed = true;
  Serial.println("  homed & cleared.");
  return true;
}

/* ================== Coordinated (simultaneous) moves ================== */
bool moveAllToStepsSimul(const long target_in[2]){
  long target[2];
  long rem[2];
  bool dirCW[2];
  unsigned long next_due[2];
  bool any_move = false;

  for (int i=0;i<2;++i){
    target[i] = target_in[i];
    if (target[i] < C[i].soft_min) target[i] = C[i].soft_min;
    if (target[i] > C[i].soft_max) target[i] = C[i].soft_max;

    long d = target[i] - S[i].steps;
    rem[i]  = d;
    dirCW[i]= (d >= 0) ? awayIsCW(i) : !awayIsCW(i); // positive d = AWAY (CW)
    if (d != 0) any_move = true;
  }

  if (!any_move){
    Serial.println("[COORD] Already at targets.");
    return true;
  }

  unsigned long now = micros();
  for (int i=0;i<2;++i) next_due[i] = now;

  Serial.print("[COORD] Move to [");
  Serial.print(target[0]); Serial.print(", "); Serial.print(target[1]); Serial.println("] steps");

  while (true){
    if (e_stop){ Serial.println("[COORD] E-STOP"); return false; }

    bool all_done = true;
    now = micros();

    for (int i=0;i<2;++i){
      long d = rem[i];
      if (d == 0) continue;
      all_done = false;

      if ((long)(now - next_due[i]) >= 0){
        if (!tryStepNoGap(i, (d >= 0) ? awayIsCW(i) : !awayIsCW(i))){
          Serial.print("[COORD] Blocked at "); Serial.println(C[i].name);
          return false;
        }
        rem[i] += (d > 0) ? -1 : +1;
        next_due[i] += (unsigned long)max(200, C[i].step_gap_us);
      }
    }

    if (all_done) break;
    delayMicroseconds(50);
  }

  Serial.println("[COORD] Done.");
  return true;
}

bool moveAllToAbsDegreesSimul(const float degs[2]){
  long target[2];
  target[0] = degToSteps(0, degs[0]);
  target[1] = degToSteps(1, degs[1]);
  return moveAllToStepsSimul(target);
}

/* ================== Initialization / Homing Sequence ==================
   EXACT ORDER:
     1) Home J1 (CCW to switch), zero, clear; then go to +180° (CW).
     2) Home J2 (CCW to switch), zero, clear; then go to +90° (CW).
     3) Store these as the Saved Home pose.
*/
bool initialize_all(){
  Serial.println("=== Init: homing J1 then J2, then move to Home (180/90) ===");

  // J1
  if (!homeOne(0)) return false;
  {
    long tgt = degToSteps(0, J1_HOME_DEG);
    long pair0[2] = { tgt, S[1].steps };           // keep J2
    if (!moveAllToStepsSimul(pair0)) return false;
  }

  // J2
  if (!homeOne(1)) return false;
  {
    long tgtJ1 = S[0].steps;                        // keep J1
    long tgtJ2 = degToSteps(1, J2_HOME_DEG);
    long pair1[2] = { tgtJ1, tgtJ2 };
    if (!moveAllToStepsSimul(pair1)) return false;
  }

  // Save Home pose (J1=180°, J2=90°)
  saved_home_steps[0] = degToSteps(0, J1_HOME_DEG);
  saved_home_steps[1] = degToSteps(1, J2_HOME_DEG);
  home_pose_saved = true;

  Serial.print("[J1] steps="); Serial.print(S[0].steps); Serial.print("  deg="); Serial.println(stepsToDeg(0),3);
  Serial.print("[J2] steps="); Serial.print(S[1].steps); Serial.print("  deg="); Serial.println(stepsToDeg(1),3);
  Serial.println("=== HOMED & READY (Home pose stored) ===");
  return true;
}

/* ================== UI / Keyboard ================== */
void printStatus(int i){
  Serial.print("["); Serial.print(C[i].name); Serial.print("] steps=");
  Serial.print(S[i].steps); Serial.print("  deg="); Serial.print(stepsToDeg(i),3);
  Serial.print("  soft:["); Serial.print(C[i].soft_min); Serial.print(","); Serial.print(C[i].soft_max); Serial.print("]");
  Serial.print("  limit="); Serial.print(stablePressed(i) ? "PRESSED":"released");
  Serial.print("  homed="); Serial.print(S[i].homed ? "yes":"no");
  Serial.print("  savedHome="); Serial.println(home_pose_saved ? "yes":"no");
}

void printAllLimits(){
  Serial.print("Limits: ");
  for (int i=0;i<2;++i){
    Serial.print(C[i].name); Serial.print("="); Serial.print(stablePressed(i)?"PRESSED ":"released ");
  }
  Serial.println();
}

void handleSerial(){
  while (Serial.available()){
    int c = Serial.read();
    if (c=='\r'||c=='\n') continue;

    if (c=='1' || c=='2'){ sel=c-'1'; jog_cmd=JOG_NONE; Serial.print("Selected "); Serial.println(C[sel].name); return; }
    if (c=='d'||c=='D'){ jog_cmd=JOG_CW;  Serial.println("CW (away from CCW limit)");  return; }
    if (c=='a'||c=='A'){ jog_cmd=JOG_CCW; Serial.println("CCW (toward CCW limit)");   return; }
    if (c==' '||c=='s'||c=='S'){ jog_cmd=JOG_NONE; e_stop = (c==' '); Serial.println(e_stop?"** E-STOP **":"STOP"); return; }
    if (c=='+'){ if (C[sel].step_gap_us>200) C[sel].step_gap_us -= 50; Serial.print("Faster gap_us="); Serial.println(C[sel].step_gap_us); return; }
    if (c=='-'){ C[sel].step_gap_us += 50; Serial.print("Slower gap_us="); Serial.println(C[sel].step_gap_us); return; }
    if (c=='p'||c=='P'){ printStatus(sel); return; }
    if (c=='l'||c=='L'){ printAllLimits(); return; }

    // 'h' -> full re-home sequence, then move to Home (180/90)
    if (c=='h'||c=='H'){
      e_stop=false;
      S[0].homed = S[1].homed = false;
      if (initialize_all()){
        Serial.println("[HOME] Completed. Home pose stored & active.");
      } else {
        Serial.println("[HOME] Failed.");
      }
      return;
    }

    // 't' -> go to Pose2 (J1=90°, J2=45°) simultaneously
    if (c=='t' || c=='T'){
      e_stop=false;
      if (!home_pose_saved){ Serial.println("[t] No home stored yet; run 'h' first."); return; }
      const float pose2_deg[2] = {J1_POSE2_DEG, J2_POSE2_DEG};
      if (!moveAllToAbsDegreesSimul(pose2_deg)){
        Serial.println("[t] Move-to-pose2 failed or aborted.");
      }
      return;
    }

    // 'u' -> return to Home (J1=180°, J2=90°) simultaneously
    if (c=='u' || c=='U'){
      e_stop=false;
      if (!home_pose_saved){ Serial.println("[u] No home stored yet; run 'h' first."); return; }
      if (!moveAllToStepsSimul(saved_home_steps)){
        Serial.println("[u] Move-to-home failed or aborted.");
      }
      return;
    }

    if (c=='e'||c=='E'){ e_stop=false; Serial.println("E-STOP cleared"); return; }
  }
}

/* ================== Arduino lifecycle ================== */
void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println("2-DOF Robot — Homing + continuous jog + global poses");
  Serial.println("Keys: 1..2 sel | d CW | a CCW | s STOP | SPACE=E-STOP | e clear | +/- speed | p status | l limits | h re-home | t pose2 | u home");

  for (int i=0;i<2;++i){
    pinMode(C[i].pul, OUTPUT);
    pinMode(C[i].dir, OUTPUT);
    digitalWrite(C[i].pul, C[i].pulse_active_high ? LOW : HIGH); // STEP idle
    pinMode(C[i].lim, INPUT_PULLUP);
    S[i].steps = 0; S[i].lastStable = false; S[i].lastRaw = false; S[i].tChange = millis();
    S[i].homed = false;
  }

  primeDebouncers(20);   // avoid nudging into the switch on boot
  printAllLimits();

  e_stop = false;
  if (initialize_all()){
    Serial.println("[BOOT] HOMED & READY — Home pose stored.");
  } else {
    Serial.println("[BOOT] Initialization FAILED. Jog disabled until homed.");
  }
}

void loop(){
  handleSerial();
  if (e_stop) { jog_cmd = JOG_NONE; return; }

  // continuous jog state machine
  if (jog_cmd != JOG_NONE){
    bool dirCW = (jog_cmd == JOG_CW);
    // If switch currently pressed, block motion TOWARD the switch (CCW)
    if (C[sel].has_limit && stablePressed(sel)){
      bool toward = (!dirCW); // since limit is at CCW, toward==CCW (dirCW=false)
      if (toward) return;
    }
    if (!stepOnce(sel, dirCW)){
      static unsigned long last=0, now; now=millis();
      if (now-last>150){ Serial.print("["),Serial.print(C[sel].name),Serial.println("] motion blocked (limit/soft cap)."); last=now; }
    } else {
      static unsigned long last=0, now; now=millis();
      if (now-last>120){
        Serial.print("["),Serial.print(C[sel].name),Serial.print("] steps="),Serial.print(S[sel].steps);
        Serial.print(" deg="),Serial.println(stepsToDeg(sel),3);
        last=now;
      }
    }
  }
}
