roscore &
#clear
sleep 1
#source /opt/ros/kinetic/setup.bash
source devel/setup.bash
#roslaunch video_stream_opencv webcam.launch camera_name:=webcam1 video_stream_provider:=0 &
#clear
#sleep 1
#roslaunch video_stream_opencv webcam.launch camera_name:=webcam2 video_stream_provider:=1 &
#clear
#sleep 1
#roslaunch video_stream_opencv webcam.launch camera_name:=webcam3 video_stream_provider:=2 &
#clear
#sleep 1
#roslaunch video_stream_opencv webcam.launch camera_name:=webcam4 video_stream_provider:=3 &
#clear
#sleep 1
#clear
#cd Detection-SARWAI
#source devel/setup.bash
roslaunch darknet_ros darknet_ros.launch &
sleep 5
#clear
roslaunch tracker tracker.launch topic_name_:=/detection/compiled_ros_msg name:=track1 &
sleep 1
#clear
roslaunch tracker tracker.launch topic_name_:=/detection/compiled_ros_msg2 name:=track2 &
sleep 1
#clear
roslaunch tracker tracker.launch topic_name_:=/detection/compiled_ros_msg3 name:=track3 &
sleep 1
#clear
roslaunch tracker tracker.launch topic_name_:=/detection/compiled_ros_msg4 name:=track4 &
sleep 1
#clear
rosrun image_draw image_draw_node &
sleep 1
#clear
rosrun detection_logger visual_logger_node


