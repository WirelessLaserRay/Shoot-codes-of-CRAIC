###gmapping with abot###
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/MH5YR2/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; source ~/MH5YR2/devel/setup.bash; roslaunch robot_slam navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; source ~/MH5YR2/devel/setup.bash; roslaunch track_tag usb_cam_with_calibration.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; source ~/MH5YR2/devel/setup.bash; roslaunch track_tag ar_track_camera.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; source ~/MH5YR2/devel/setup.bash; roslaunch abot_bringup shoot.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; source ~/MH5YR2/devel/setup.bash; rosrun robot_slam nav_end.py; exec bash"' \
--tab -e 'bash -c "sleep 8; source ~/MH5YR2/devel/setup.bash; rosrun robot_voice tts_subscribe; exec bash"' \
--tab -e 'bash -c "cd /home/abot/MH5YR2/src/robot_slam/scripts/; python3 demo.py; exec bash"' \
--tab -e 'bash -c "sleep 8; source ~/MH5YR2/devel/setup.bash; roslaunch robot_slam multi_goal.launch; exec bash"' \
# --tab -e 'bash -c "sleep 3; source ~/MH5YR2/devel/setup.bash; roslaunch find_object_2d find_object_2d.launch; exec bash"' \



