FROM osrf/ros:humble-desktop-full-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential git python3-pip python3-colcon-common-extensions \
      python3-rosdep python3-pytest \
      ros-humble-moveit ros-humble-gazebo-ros2-control \
      ros-humble-ros2-control ros-humble-ros2-controllers \
      ros-humble-ros2controlcli ros-humble-moveit-servo \
      libmongoclient-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/mycobot_ws
# Match the warehouse dependency already validated by this project.
ARG WAREHOUSE_COMMIT=32f8fc5dd245077b9c09e93efc8625b9f599f271
RUN git init src/warehouse_ros_mongo \
    && git -C src/warehouse_ros_mongo remote add origin https://github.com/ros-planning/warehouse_ros_mongo.git \
    && git -C src/warehouse_ros_mongo fetch --depth 1 origin "$WAREHOUSE_COMMIT" \
    && git -C src/warehouse_ros_mongo checkout --detach FETCH_HEAD

COPY requirements.txt /tmp/mycobot-requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/mycobot-requirements.txt
COPY mycobot_description/ src/mycobot/mycobot_description/
COPY mycobot_280/ src/mycobot/mycobot_280/
RUN source /opt/ros/humble/setup.bash \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update --rosdistro humble \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro humble -y \
    && rm -rf /var/lib/apt/lists/*

# Limit compiler memory usage on ordinary laptops; override at build time.
ARG BUILD_JOBS=2
RUN source /opt/ros/humble/setup.bash \
    && MAKEFLAGS="-j${BUILD_JOBS}" colcon build --executor sequential \
       --packages-up-to mycobot_280_hardware mycobot_280_pick_place_demo \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

COPY docker/entrypoint.sh /mycobot-entrypoint.sh
RUN chmod +x /mycobot-entrypoint.sh
ENV ROS_DOMAIN_ID=52 QT_X11_NO_MITSHM=1
ENTRYPOINT ["/mycobot-entrypoint.sh"]
CMD ["ros2", "launch", "mycobot_280_hardware", "control.launch.py", "adaptive_gripper:=true", "launch_driver:=false"]
