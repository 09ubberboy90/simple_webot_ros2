ARG BASE_IMAGE=nvidia/cudagl:11.0-devel-ubuntu20.04

FROM ${BASE_IMAGE} AS base

ENV USER_NAME ubb
ENV USER_PASSWORD docker
ENV DEBIAN_FRONTEND noninteractive

ENV ARGS " "
RUN apt-get update && \
  apt-get install -y sudo \
  software-properties-common \
  curl \
  git-core \
  gnupg \
  zsh \
  wget \
  nano \
  xvfb \
  fonts-powerline \
  gnupg2 \
  lsb-release \
  # add a user (--disabled-password: the user won't be able to use the account until the password is set)
  && adduser --quiet --disabled-password --shell /bin/zsh --home /home/$USER_NAME --gecos "User" $USER_NAME \
  # update the password
  && echo "${USER_NAME}:${USER_PASSWORD}" | chpasswd && usermod -aG sudo $USER_NAME
  
FROM base AS ros2

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -


RUN apt-get update
RUN apt-get install -y ros-foxy-desktop \
  ros-foxy-moveit \
  ros-foxy-moveit-common \
  ros-foxy-gripper-controllers \
  ros-foxy-ros2-controllers \
  ros-foxy-ros2-control 

RUN apt-get install -y python3-rosdep \
    python3-pip \
    python3-colcon-common-extensions \ 
    python3-colcon-mixin \
    ros-foxy-gazebo-msgs \
    ros-foxy-xacro \
    ros-foxy-warehouse-ros-mongo \
    ros-foxy-backward-ros

RUN pip3 install -U pytest pyquaternion

FROM ros2 AS webots

ARG WEBOTS_VERSION=R2021b
ARG WEBOTS_PACKAGE_PREFIX=
# Install Webots runtime dependencies
RUN apt update && rm -rf /var/lib/apt/lists/
RUN wget https://raw.githubusercontent.com/cyberbotics/webots/master/scripts/install/linux_runtime_dependencies.sh
RUN chmod +x linux_runtime_dependencies.sh && ./linux_runtime_dependencies.sh && rm ./linux_runtime_dependencies.sh && rm -rf /var/lib/apt/lists/

# Install X virtual framebuffer to be able to use Webots without GPU and GUI (e.g. CI)
RUN apt update && apt install --yes xvfb && rm -rf /var/lib/apt/lists/

# Install Webots
WORKDIR /usr/local
RUN wget https://github.com/cyberbotics/webots/releases/download/$WEBOTS_VERSION/webots-$WEBOTS_VERSION-x86-64$WEBOTS_PACKAGE_PREFIX.tar.bz2
RUN tar xjf webots-*.tar.bz2 && rm webots-*.tar.bz2

RUN apt update && apt install -y ros-foxy-webots-ros2
ENV WEBOTS_HOME /usr/local/webots
ENV PATH=/usr/local/webots:$PATH
FROM webots AS sim_comp

RUN sudo rosdep init
RUN rosdep update

RUN cd /home/${USER_NAME}/ \
    && mkdir -p /home/${USER_NAME}/src \
    && chown ${USER_NAME}:${USER_NAME} -R /home/${USER_NAME}\
    && cd /home/${USER_NAME}/src \
    && git clone https://github.com/09ubberboy90/simple_webot_ros2.git \
    && rosdep install -r --from-paths . --ignore-src --rosdistro foxy -y

RUN cd /home/${USER_NAME}/ \
    && . /opt/ros/foxy/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && chown ${USER_NAME}:${USER_NAME} -R /home/${USER_NAME}


COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh 
USER $USER_NAME

ENTRYPOINT ["/entrypoint.sh"]

CMD ["sh", "-c", "ros2 run sim_recorder run_recording ${ARGS}"] 


