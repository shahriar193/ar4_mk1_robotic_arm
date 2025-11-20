# initialize_once.py
from arm2d import Arm2D

# Tip: pass port="/dev/cu.usbmodemXXXX" if auto-detect fails on your Mac.
arm = Arm2D()

print("Initializing (homing all joints)...")
state = arm.initialize()  # sends "h" and then "p" to read state
print("✅ Initialized. Current pose:", state)

arm.close()
