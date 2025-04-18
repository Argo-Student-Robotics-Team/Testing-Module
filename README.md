# Testing-Module
Module designed to test the functionality of rover components<br>

Realsense_tester: Testing if the camera is connected and has proper depth and color streaming and distance<br>
Running: In onde terminal pass: source /opt/ros/humble/setup.bash<br>
				       ros2 launch realsense2_camera rs_launch.py<br>
	 In second terminal pass(after colcon building): source /opt/ros/humble/setup.bash<br>
			                                 ros2 run realsense_tester realsense_test_node<br>
-----------------------------------------------------------------------------------------------------------
