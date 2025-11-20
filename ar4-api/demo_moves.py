# demo_moves.py
import time
from arm2d import Arm2D

arm = Arm2D()

# If the robot was just powered on, run initialize() once.
# If it's already homed and still powered, you can skip this line.
# arm.initialize()

# print("Move by IK to x=120, y=-80, z=150...")
res = arm.move_xyz(120, -80, 150)
print("\n".join(res.reply))

time.sleep(0.5)

# # print("Move by math angles: J1=30°, J2=40°...")
# res = arm.move_math(30, 50)
# print("\n".join(res.reply))

res = arm.move_xyz(82, 0, 250)
print("\n".join(res.reply))

time.sleep(2.0)

res = arm.move_xyz(119, -95, 150)
print("\n".join(res.reply))

time.sleep(2.0)

res = arm.move_xyz(126, 42, 190)
print("\n".join(res.reply))

time.sleep(2.0)

res = arm.move_xyz(150, -52, 134)
print("\n".join(res.reply))

time.sleep(2.0)

res = arm.move_xyz(127, 0, 200)
print("\n".join(res.reply))

time.sleep(2.0)

res = arm.move_xyz(118, -47, 200)
print("\n".join(res.reply))

time.sleep(2.0)

res = arm.move_xyz(97, 32, 230)
print("\n".join(res.reply))

time.sleep(2.0)


print("Status:")
st = arm.status()
print(st["parsed"])

arm.close()
