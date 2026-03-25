# Test of the bluerov2_motion_control package
If you want to test whether all the nodes of the bluerov2_motion_control pkg are working correctly, you can launch the associated launch file, and simulate information coming from Qualisys. 

Launch the following commands:

```
rostopic pub -r 10 /bluerov2_heavy/reference_position/linear/x std_msgs/Float64 "data: 1.5"
rostopic pub -r 10 /bluerov2_heavy/reference_position/linear/y std_msgs/Float64 "data: 0.0"
rostopic pub -r 10 /bluerov2_heavy/reference_position/linear/z std_msgs/Float64 "data: 0.3"
rostopic pub -r 10 /bluerov2_heavy/reference_position/angular/z std_msgs/Float64 "data: 0.1"

rostopic pub -r 10 /bluerov2_heavy/position/linear/x std_msgs/Float64 "data: 0.3"
rostopic pub -r 10 /bluerov2_heavy/position/linear/y std_msgs/Float64 "data: 0.4"
rostopic pub -r 10 /bluerov2_heavy/position/linear/z std_msgs/Float64 "data: 0.25"
rostopic pub -r 10 /bluerov2_heavy/position/angular/z std_msgs/Float64 "data: 0.3"


rostopic pub -r 10 /bluerov2_heavy/velocity/linear/x std_msgs/Float64 "data: 0.3"
rostopic pub -r 10 /bluerov2_heavy/velocity/linear/y std_msgs/Float64 "data: 0.3"
rostopic pub -r 10 /bluerov2_heavy/velocity/linear/z std_msgs/Float64 "data: 0.3"
rostopic pub -r 10 /bluerov2_heavy/velocity/angular/z std_msgs/Float64 "data: 0.3"
```

You can now test how the control system responds, for instance, by running:
```
rostopic echo /bluerov2_heavy/reference_velocity/linear/x
rostopic echo /bluerov2_heavy/cmd_velocity/linear/x
```

As a further check, you can run rqt_graph, which will return:  
<img src="https://github.com/guobang494/BlueROV2H-SimTank-ROS-Qualisys/blob/main/INSTALLATION/unit_testing/rqt_graph_bluerov_motion_control.png" width=100% height=100%>
  
  
