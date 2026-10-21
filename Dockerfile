# Use the official ROS 2 Humble desktop image
FROM osrf/ros:humble-desktop

# Set the working directory
WORKDIR /workspace

# Install additional dependencies if needed
# For example, you can uncomment the line below to install a package
# Install additional dependencies
RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
    build-essential \
    gcc \
    g++ \
    git \
    libarmadillo-dev \
    libeigen3-dev \
    libxml2-dev \
    libxslt-dev \
    libpcl-dev \
    python3 \
    python3-colcon-common-extensions \
    python3-pip \
    python3-scipy

# Copy the colcon workspace (the repository root) into the image
COPY . /workspace/

WORKDIR /workspace


# Set environment variables
ENV ROS_DISTRO humble
ENV ROS_VERSION 2
# Build all packages with colcon (CMake defaults to Release)
RUN . /opt/ros/humble/setup.sh && colcon build


# Source the workspace setup file
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Set entry point to start a bash shell in the built workspace
CMD ["bash"]
