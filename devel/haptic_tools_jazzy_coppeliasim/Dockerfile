# CoppeliaSim 4.7 (rev4) image to run the scenes in scenes/. The base image already provides ROS 2 Jazzy, SAS (with
# sas_robot_driver_coppeliasim, used by the sas_object_script of the scenes), DQ Robotics, the simROS2 plugin, and the
# Python packages (pyzmq, cbor2) that CoppeliaSim needs for Python child scripts. The script of the holonomic base of
# haptic_scene.ttt only needs rclpy and geometry_msgs, also in the base image.
#
# Build, e.g. from the repository root:
#   docker build -f devel/haptic_tools_jazzy_coppeliasim/Dockerfile -t haptic_tools_jazzy_coppeliasim .
FROM murilomarinho/coppeliasim:noble_470rev4
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

# Interactive shells (e.g. terminator) get the same environment as non-interactive ones (BASH_ENV).
RUN grep -q "source /etc/bash_env" ~/.bashrc || echo "source /etc/bash_env" >> ~/.bashrc

########################################################################################################
# The scenes are mounted here by the compose file (not copied), so that the changes saved from CoppeliaSim
# land in the repository.
RUN mkdir -p /app/scenes
WORKDIR /app/scenes
