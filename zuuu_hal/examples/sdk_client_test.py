from reachy2_sdk import ReachySDK
import time

reachy = ReachySDK(host='localhost') 

print(reachy.mobile_base)

print("turning on mobile base...")
reachy.mobile_base.turn_on()
print("resetting odometry...")
reachy.mobile_base.reset_odometry()
time.sleep(1.0)
# reachy.mobile_base.goto(x=1.0, y=0.0, theta=0.0, timeout=10000, tolerance={"delta_x": 0.0, "delta_y": 0.0, "delta_theta": 0.0, "distance": 0.0})
# exit()
print("Drawing a square of 1x1m")
# while True :
#     reachy.mobile_base.goto(x=1.0, y=0.0, theta=0.0, timeout=10000, tolerance={"delta_x": 0.05, "delta_y": 0.05, "delta_theta": 5.0, "distance": 0.05})
#     reachy.mobile_base.goto(x=1.0, y=1.0, theta=90.0, timeout=10000, tolerance={"delta_x": 0.05, "delta_y": 0.05, "delta_theta": 5.0, "distance": 0.05})
#     reachy.mobile_base.goto(x=0.0, y=1.0, theta=180.0, timeout=10000, tolerance={"delta_x": 0.05, "delta_y": 0.05, "delta_theta": 5.0, "distance": 0.05})
#     reachy.mobile_base.goto(x=0.0, y=0.0, theta=270.0, timeout=10000, tolerance={"delta_x": 0.05, "delta_y": 0.05, "delta_theta": 5.0, "distance": 0.05})

# while True :
#     reachy.mobile_base.goto(x=1.0, y=1.0, theta=90.0, timeout=10000, tolerance={"delta_x": 0.05, "delta_y": 0.05, "delta_theta": 5.0, "distance": 0.05})
#     reachy.mobile_base.goto(x=0.0, y=0.0, theta=0.0, timeout=10000, tolerance={"delta_x": 0.05, "delta_y": 0.05, "delta_theta": 5.0, "distance": 0.05})
    

while True :
    print("goto!")
    reachy.mobile_base.goto(x=1.0, y=1.0, theta=90.0, timeout=10000, tolerance={"delta_x": 0.05, "delta_y": 0.05, "delta_theta": 5.0, "distance": 0.05})
    print("resetting odometry...")
    
    reachy.mobile_base.reset_odometry()
    time.sleep(0.1)
    
    print("anti goto!")
    reachy.mobile_base.goto(x=-1.0, y=1.0, theta=-90.0, timeout=10000, tolerance={"delta_x": 0.05, "delta_y": 0.05, "delta_theta": 5.0, "distance": 0.05})
    print("resetting odometry...")
    reachy.mobile_base.reset_odometry()
    time.sleep(0.1)
    





# input("reachy.mobile_base.goto(x=0.0, y=0.0, theta=-90.0)...")
# reachy.mobile_base.goto(x=1.0, y=1.0, theta=-30.0, timeout=10000, tolerance={"delta_x": 0.05, "delta_y": 0.05, "delta_theta": 5.0, "distance": 0.05})
# input("resetting odometry...")
# reachy.mobile_base.reset_odometry()
# input("reachy.mobile_base.goto(x=0.0, y=0.0, theta=-90.0)...")
# reachy.mobile_base.goto(x=1.0, y=1.0, theta=-30.0, timeout=10000, tolerance={"delta_x": 0.05, "delta_y": 0.05, "delta_theta": 5.0, "distance": 0.05})



input("reachy.mobile_base.goto(x=0.0, y=0.0, theta=90.0)...")
reachy.mobile_base.goto(x=0.0, y=0.0, theta=90.0, timeout=10000, tolerance={"delta_x": 0.01, "delta_y": 0.01, "delta_theta": 1.0, "distance": 0.01})

input("Go back to 0 degrees")
reachy.mobile_base.goto(x=0.0, y=0.0, theta=0.0, timeout=10000, tolerance={"delta_x": 0.01, "delta_y": 0.01, "delta_theta": 1.0, "distance": 0.01})
input("reachy.mobile_base.goto(x=0.0, y=0.0, theta=-90.0)...")
reachy.mobile_base.goto(x=0.0, y=0.0, theta=-90.0, timeout=10000, tolerance={"delta_x": 0.01, "delta_y": 0.01, "delta_theta": 1.0, "distance": 0.01})

input("resetting odometry...")
reachy.mobile_base.reset_odometry()
input("Going back to 0 degrees in the frame : it won't move because the frame has changed")
reachy.mobile_base.goto(x=0.0, y=0.0, theta=0.0, timeout=10000, tolerance={"delta_x": 0.01, "delta_y": 0.01, "delta_theta": 1.0, "distance": 0.01})


input("reachy.mobile_base.goto(x=0.0, y=0.0, theta=-90.0)...")
reachy.mobile_base.goto(x=0.0, y=0.0, theta=-90.0, timeout=10000, tolerance={"delta_x": 0.01, "delta_y": 0.01, "delta_theta": 1.0, "distance": 0.01})