# Build the workspace with colcon
.PHONY : build
build:
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Clean the colcon build artifacts
.PHONY : clean
clean:
	rm -rf build install log

# Build docker project
.PHONY : docker_build
docker_build:
	docker build -t sf .

# Run docker project
.PHONY : docker_run
docker_run:
	docker run -it --network host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1"  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --entrypoint /bin/bash sf


