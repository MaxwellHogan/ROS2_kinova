
Build the docker container:

docker build --no-cache -t ros2_kinova -f docker/Dockerfile.kinova .

run your built container on windows:
.\scripts\run_kinova_container.bat 

run your built container on MacOS:
.\scripts\run_kinova_container.sh 

