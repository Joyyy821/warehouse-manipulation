启动真实机器人
roslaunch interbotix_moveit interbotix_moveit.launch robot_name:=wx250s use_actual:=true dof:=6

运行例程
roslaunch interbotix_demos moveit_XXXXXX.launch robot_name:=wx250s 

rosrun interbotix_demos moveit_fk_demo.py __ns:=wx250s


Gazebo仿真
roslaunch interbotix_moveit interbotix_moveit.launch robot_name:=wx250s use_gazebo:=true dof:=6

需要点击开始仿真


moveit
roslaunch interbotix_moveit interbotix_moveit.launch robot_name:=wx250s use_fake:=true dof:=6

视觉抓取
roslaunch object_color_detector usb_cam.launch
rqt_image_view

./para_test image.png


sudo apt install python3-sklearn
roslaunch interbotix_moveit interbotix_moveit.launch robot_name:=wx250s use_actual:=true dof:=6
roslaunch object_color_detector object_detect_hsv.launch
roslaunch object_color_detector camera_calibration_hsv.launch __ns:=wx250s


roslaunch interbotix_moveit interbotix_moveit.launch robot_name:=wx250s use_actual:=true dof:=6
roslaunch object_color_detector object_detect_hsv.launch
rosrun interbotix_demos vision_grasp_demo.py __ns:=wx250s
