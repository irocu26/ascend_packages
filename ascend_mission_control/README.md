## build the workspace properly 

## Run the normal gz ardupilot simulation
``` bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

## check the topic list
``` bash
ros2 topic list #should contain /ap topics
```
## Check apinterface node
The iris should takeoff and hover for 5 seconds and move with 0.5m/s velocity and then land
``` bash
ros2 run ascend_mission_control ap_interface
```