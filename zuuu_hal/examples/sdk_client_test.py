from reachy2_sdk import ReachySDK
import time

reachy = ReachySDK(host='localhost') 

print(reachy.mobile_base)

print("turning on mobile base...")
reachy.mobile_base.turn_on()
print("resetting odometry...")
reachy.mobile_base.reset_odometry()
time.sleep(1.0)


input("Press Enter to make the robot move forward for 2 seconds")
reachy.mobile_base.goto(x=20.0, y=0.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
time.sleep(2.0)
print("Turn off!")
reachy.mobile_base.turn_off()

input("Press Enter to request a goto x=0.0, y=0.0, theta=00.0, nothing should happen...")
print("Requesting another goto, the robot should not move...")
reachy.mobile_base.goto(x=0.0, y=0.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)

input("Press Enter to turn ON the mobile base and go back to the initial position")
print("Turn on!")
reachy.mobile_base.turn_on()
reachy.mobile_base.goto(x=0.0, y=0.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)



input("Press Enter to draw a square of 1x1m")
reachy.mobile_base.goto(x=1.0, y=0.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
reachy.mobile_base.goto(x=1.0, y=1.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
reachy.mobile_base.goto(x=0.0, y=1.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
reachy.mobile_base.goto(x=0.0, y=0.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
print("This message should appear before the end of the square.")


input("Press Enter to draw a square of 1x1m with weird rotations")

reachy.mobile_base.goto(x=1.0, y=0.0, theta=90.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
reachy.mobile_base.goto(x=1.0, y=1.0, theta=-90.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
reachy.mobile_base.goto(x=0.0, y=1.0, theta=-180.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
reachy.mobile_base.goto(x=0.0, y=0.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
print("This message should appear before the end of the square. ")


input("Press Enter to rotate 720° back and forth once")


reachy.mobile_base.goto(x=0.0, y=0.0, theta=720.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)
reachy.mobile_base.goto(x=0.0, y=0.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=10000)

input("Press Enter to test the timeout. If it fails the robot will move forward 5 meters :) If it works, it will move forward during 2 seconds and then come back")


print("Testing the timeout, the robot should not reach x=10.0")
reachy.mobile_base.goto(x=5.0, y=0.0, theta=00.0, wait=False, degrees=True, distance_tolerance=0.05, angle_tolerance=5.0, timeout=2)
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
