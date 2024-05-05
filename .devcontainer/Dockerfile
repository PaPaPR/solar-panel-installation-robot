# ROS distribution to use
ARG ROS_DISTRO=humble

##############
# Base Image #
##############
FROM osrf/ros:${ROS_DISTRO}-desktop-full as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
  git python-is-python3 wget unzip \
  && rm -rf /var/lib/apt/lists/*

# Create workspace with external dependencies
RUN mkdir -p /underlay_ws/src
WORKDIR /underlay_ws/src

# Build the base workspace, installing dependencies first.
WORKDIR /underlay_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Set up the entrypoint
WORKDIR /underlay_ws
COPY ./.devcontainer/entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

#################
# Overlay Image #
#################
FROM base as overlay

# Apriltag installation
WORKDIR /tmp
RUN git clone https://github.com/AprilRobotics/apriltag.git --depth 1
WORKDIR /tmp/apriltag
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release
RUN cmake --build build --target install

# Create an overlay workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws
COPY ./src/ ./src/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && source /underlay_ws/install/setup.bash \
 && colcon build

# Set up the entrypoint
# COPY ./entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#####################
# Development Image #
#####################
FROM overlay as dev

# Dev container arguments
ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

# Install extra tools for development
RUN apt-get update && apt-get install -y --no-install-recommends \
  gdb gdbserver nano \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-foxglove-bridge

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /overlay_ws/

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
# RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc
RUN echo -e "source /opt/ros/${ROS_DISTRO}/setup.bash\n \
  source /underlay_ws/install/setup.bash\n \
  source /overlay_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc
ENTRYPOINT [ "/entrypoint.sh" ]