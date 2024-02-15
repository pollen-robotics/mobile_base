## Purpose
Configuration files to launch nav2 on the mobile base Zuuu.
This was used under Foxy, not yet ported to Humble/iron.

## Usage
For a simulated environment:
```
ros2 launch zuuu_description full_simulation_navigation.launch.py
```

To create a map on the physical robot:
```
ros2 launch zuuu_description zuuu_full_navigation.launch.py
```

To start the navigation on the physical robot, if your map name is jean_jaures_haut.yaml, add this to your bashrc:
```
export ZUUU_MAP_NAME=jean_jaures_haut
```
Then:
```
ros2 launch zuuu_description zuuu_full_navigation.launch.py
```

## List of launch files and uses
Launch files starting with "zuuu" are intended to be used on the physical robot.


### full_simulation_navigation
Launches: 
gazebo_simulation
navigation
rviz_navigation

### mapping
Nodes:
slam_toolbox

### navigation
Launches:
Nav2's bringup_launch


### zuuu_full_mapping
Launches:
zuuu_bringup
mapping

### zuuu_full_navigation
Launches:
zuuu_bringup
navigation