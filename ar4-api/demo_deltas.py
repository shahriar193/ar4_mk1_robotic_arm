# demo_deltas.py
from arm2d import Arm2D
import time

arm = Arm2D()          # set port="COM5" or "/dev/cu.usbmodemXXXX" if auto-detect fails

# Run once after power-on (skip if already homed and still powered)
# arm.initialize()

print("Bump +10° on J1 (keep J2 same)...")
res1 = arm.move_delta_math(dJ1=-20, dJ2=None)
print("\n".join(res1.reply))

time.sleep(0.3)

print("Bump +10° on J1 (keep J2 same)...")
res1 = arm.move_delta_math(dJ1=20, dJ2=None)
print("\n".join(res1.reply))

time.sleep(0.3)

print("Bump 20° on J2 only...")
res2 = arm.move_delta_math(dJ2=20)  # J1 unchanged (None by default)
print("\n".join(res2.reply))

time.sleep(0.3)

print("Bump -20° on J2 only...")
res2 = arm.move_delta_math(dJ2=-20)  # J1 unchanged (None by default)
print("\n".join(res2.reply))

time.sleep(0.3)

print("Nudge +40 mm in X, keep Y/Z unchanged...")
res3 = arm.move_delta_xyz(dx=40)     # dy/dz default to None → unchanged
print("\n".join(res3.reply))

time.sleep(0.3)

print("Nudge -40 mm in Z only...")
res4 = arm.move_delta_xyz(dz=-40)
print("\n".join(res4.reply))

print("Nudge 40 mm in Y only...")
res4 = arm.move_delta_xyz(dy=40)
print("\n".join(res4.reply))

print("Final status:")
print(arm.status()["parsed"])

arm.close()
