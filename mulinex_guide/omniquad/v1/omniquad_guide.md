# How to Use OmniQuad - A Step-by-Step Guide

1. Put the robot in the ‘bearskin’ configuration
   
   <img src="img/bearskin_1.jpg" alt="Bearskin configuration" width="300"/>

   <img src="img/bearskin_2.jpg" alt="Bearskin configuration" width="300"/>
2. Turn on the four switches on the robot
3. Connect to the WiFi network `MulsbcWifi` (Password: `1234567890`)
4. Open a terminal and run:
   ```bash
   ssh mulsbc@192.168.10.1
   ```
   Write the password `123456789` when prompted.
   ```bash
   ssh mulsbc@192.168.10.1 #(Password 123456)
   sudo su
   cd mulinex_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   export ROS_DOMAIN_ID=1
   ros2 launch pi3hat_hw_interface start_MJBots_Pi3Hat_hw_Interface.py urdf_file:=omni_mulinex.xacro conf_file:=omniquad_joy.yaml
   ```
5. Connect the joystick to the PC
6. Open *another* terminal and run:
   ```bash
   cd Documents/mulinex_ws/
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   export ROS_DOMAIN_ID=1
   ros2 launch omni_mulinex_joystic start_joystic_nodes.launch.py
   ```
8. Press X or A to start robot homing
9. Press the O or B key to disconnect the controllers (robot falls, hold it or put it on its altar)
10. Enjoy :)
