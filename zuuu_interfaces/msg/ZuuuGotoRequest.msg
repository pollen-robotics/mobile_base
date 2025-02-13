# x goal in odom frame (m)
float64 x_goal
# y goal in odom frame (m)
float64 y_goal
# theta goal in odom frame (rad)
float64 theta_goal

# Tolerance for the distance (m) and angle (rad)
float64 dist_tol
float64 angle_tol

# Timeout (s)
float64 timeout

# When the goal is reached, should the robot keep trying to reach it or stop all commands?
bool keep_control_on_arrival true

# PID tunning for the distance controller
float64 distance_p 5.0
float64 distance_i 0.0
float64 distance_d 0.0
float64 distance_max_command 0.4

# PID tunning for the angle controller
float64 angle_p 5.0
float64 angle_i 0.0
float64 angle_d 0.0
float64 angle_max_command 1.0