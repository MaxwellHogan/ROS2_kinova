@echo on
docker run -it ^
           --rm ^
           --name ros2_dev_container ^
           -e DISPLAY=host.docker.internal:0 ^
           -v "C:\ROS2_kinova\workspace\":/ros2_ws ^
           ros2_kinova
pause
