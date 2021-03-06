# Leo-Drive-Control-Engineer-Assignment

Description: Simulation of a longitudinal vehicle controller for speed control by using PID technique.

Followed Steps:

1 - I installed Ubuntu 20.04 operating system by dual booting my computer.

2 - I installed ROS2 Foxy

3 - I installed the SVL Simulator, opened my own account and defined my cluster. I selected the following options to     test the "Speed Control" scenario.

Assets
    
    Map:	Straight1LaneSame
    Vehicle
      Asset:	Lexus2016RXHybrid
      Sensor configuration:	Autoware.Auto
    Bridge:   ROS2
    Autopilot:	Other ROS 2 Autopilot
    Connection:	localhost:9090
    
Test Case

    Runtime Template:	Random Traffic
    Time of Day:	12:00
    Rain:	0
    Fog:	0
    Wetness:	0
    Cloudiness:	0
    Random traffic:	false
    Predefined Random Seed:	0
    Random pedestrians:	false
    Random bicyclists:	false
    
4 - I installed the ROS2 bridge package, which allows me to access the topics of the SVL Simulator.
 
5 - I installed the LGSVL messages package.
 
6 - I installed the PlotJuggler plugin.

7 - I created a C++ package.
 
8 - To create the float32 acceleration_pct (0 to 1) and float32 braking_pct (0 to 1) inputs in the interface of the lgsv/vehicle_ control_cmd topic
    lgsvl_msgs/msg/Vehicle ControlData, the type of the /lgsv/state_report topic lgsvl_msgs/msg I implemented PID control algorithm using the float32 speed_mps
    output in the interface of /CanBusData in a node written in C++.
     
9 - I compiled the package.

10 - I started the "lgsvl_bridge" to communicate with the SVL Simulator.

11 - I ran my simulation.

12 - I run the "speed_controller" node in the package that I created.

Result:

I have successfully wrote the C++ code "longitudinal_vehicle_speed_controller_node.cpp" for the node "speed_control!
ler". However, as I can't understand why, the car brakes as soon as the control system activates.

Screenshots:

![Screenshot from 2021-11-08 20-59-20](https://user-images.githubusercontent.com/94018630/141017032-ee4941e4-b027-401c-af26-750eda15d65f.png)
![Screenshot from 2021-11-10 01-56-15](https://user-images.githubusercontent.com/94018630/141019291-97aa94fe-0a21-49ac-a8b9-2da13b24a9c6.png)
![Screenshot from 2021-11-10 01-56-31](https://user-images.githubusercontent.com/94018630/141019296-814bacde-0d80-440f-abf0-a83888567513.png)
![Screenshot from 2021-11-10 01-57-10](https://user-images.githubusercontent.com/94018630/141019309-a0341b70-57b6-4f81-948f-3df85e67b0ec.png)

