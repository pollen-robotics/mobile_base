from reachy2_sdk import ReachySDK
import time

reachy = ReachySDK(host='localhost') 

print(reachy.mobile_base)

print("turning on mobile base...")
reachy.mobile_base.turn_on()
print("resetting odometry...")
reachy.mobile_base.reset_odometry()
time.sleep(1.0)

print("Drawing a square of 1x1m with wait=False")
reachy.mobile_base.goto(x=1.0, y=0.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
reachy.mobile_base.goto(x=1.0, y=1.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
reachy.mobile_base.goto(x=0.0, y=1.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
reachy.mobile_base.goto(x=0.0, y=0.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
print("This message should appear before the end of the square. Waiting with time.sleep(5.0)")

time.sleep(5.0)

print("Testing the timeout, the robot should not reach x=10.0")
reachy.mobile_base.goto(x=10.0, y=0.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=2)
reachy.mobile_base.goto(x=0.0, y=0.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)

print("Note: the timeout had an effect because there was another goto scheduled, otherwise the mobile base would continue to go to x=10.0. -> Validated with Gaëlle.")

# TODO Remi x Team interface, wait=True fails



# print("Rotating 90° back and forth")

# reachy.mobile_base.goto(x=0.0, y=0.0, theta=90.0, wait=True, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
# reachy.mobile_base.goto(x=0.0, y=0.0, theta=00.0, wait=True, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)

# print("Drawing a square of 1x1m with wait=True")

# reachy.mobile_base.goto(x=1.0, y=0.0, theta=00.0, wait=True, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
# reachy.mobile_base.goto(x=1.0, y=1.0, theta=00.0, wait=True, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
# reachy.mobile_base.goto(x=0.0, y=1.0, theta=00.0, wait=True, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
# reachy.mobile_base.goto(x=0.0, y=0.0, theta=00.0, wait=True, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
# print("finished square")


# TODO test all the other parameters


exit()
