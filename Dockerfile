FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive \
    TURTLEBOT3_MODEL=burger \
    QT_X11_NO_MITSHM=1

# Add network timeout and retries so apt-get never hangs indefinitely
RUN echo 'Acquire::http::Timeout "30";' > /etc/apt/apt.conf.d/99timeout && \
    echo 'Acquire::Retries "3";' >> /etc/apt/apt.conf.d/99timeout

RUN apt-get update && apt-get install -y --no-install-recommends \
     ros-jazzy-turtlebot3-gazebo \
     ros-jazzy-rviz2 \
     ros-jazzy-turtlebot3-teleop \
     ros-jazzy-turtlebot3-navigation2 \
     ros-jazzy-nav2-bringup \
     ros-jazzy-ros-gz-sim \
     ros-jazzy-ros-gz-bridge \
     ros-jazzy-ament-cmake-gtest \
     libcurl4-openssl-dev nlohmann-json3-dev \
     python3-yaml python3-pip \
     xvfb mesa-utils x11-apps \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY . src/pose_saver_pkg
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash" >> /root/.bashrc
CMD ["bash", "-c", "source /ros2_ws/install/setup.bash && exec ros2 launch pose_saver_pkg pose_saver_turtlebot.launch.py"]
