# The packages of this repository (haptic_tools_core, haptic_tools_msgs, haptic_navigation, haptic_simulator, and the
# examples), built in a ROS 2 workspace. The base image already provides ROS 2 Jazzy, SAS, and DQ Robotics.
#
# Build, e.g. from the repository root:
#   docker build -f devel/haptic_tools_jazzy/Dockerfile -t haptic_tools_jazzy .
FROM murilomarinho/sas:jazzy
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"
RUN echo "export ROS_DOMAIN_ID=77" >> /etc/bash_env

########################################################################################################
# Update the SAS packages of the base image. Only those already installed: the wildcard also matches packages,
# such as ros-jazzy-sas-robot-driver-gazebo, whose dependencies are not available here. The debug symbols are
# removed first, as they pin the old versions (the repository does not always have them for the new ones).
RUN apt-get update && \
    apt-get purge -y 'ros-jazzy-sas-*-dbgsym' && \
    apt-get install -y --only-upgrade 'ros-jazzy-sas-*' && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

########################################################################################################
# The receiver of the haptic device (Touch X), and Qt6 for haptic_simulator and the GUI of
# haptic_navigation_example. sas_operator_side_receiver is in the noncommercial (CC BY-NC) repository of
# SmartArmStack, which the base image does not have.
RUN curl -fsSL https://marinholab.github.io/sas_debian_builder_noncommercial/KEY.gpg | \
        gpg --dearmor -o /etc/apt/trusted.gpg.d/smartarmstack_cc_by_nc.gpg && \
    curl -fsSL https://marinholab.github.io/sas_debian_builder_noncommercial/smartarmstack_cc_by_nc.list \
        -o /etc/apt/sources.list.d/smartarmstack_cc_by_nc.list
RUN apt-get update && \
    apt-get install -y \
        ros-jazzy-sas-operator-side-receiver \
        qt6-base-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

########################################################################################################
# Copy the packages of this repository (not the scenes, nor devel/) into the ROS 2 workspace, and build them.
RUN mkdir -p /app/ros2_ws/src/haptic_tools
COPY ./haptic_tools_core /app/ros2_ws/src/haptic_tools/haptic_tools_core
COPY ./haptic_tools_msgs /app/ros2_ws/src/haptic_tools/haptic_tools_msgs
COPY ./haptic_navigation /app/ros2_ws/src/haptic_tools/haptic_navigation
COPY ./haptic_simulator /app/ros2_ws/src/haptic_tools/haptic_simulator
COPY ./examples /app/ros2_ws/src/haptic_tools/examples

RUN echo "alias cdros2='cd /app/ros2_ws/' " >> /etc/bash_env
RUN echo "alias buildros2='cdros2 && colcon build && source install/setup.bash' " >> /etc/bash_env

RUN source /opt/ros/jazzy/setup.bash && \
    cd /app/ros2_ws && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /app/ros2_ws/install/setup.bash" >> /etc/bash_env

# Interactive shells (e.g. terminator) get the same environment as non-interactive ones (BASH_ENV).
RUN echo "source /etc/bash_env" >> ~/.bashrc
WORKDIR /app/ros2_ws
