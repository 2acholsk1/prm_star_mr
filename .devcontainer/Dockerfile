FROM ros:humble
ARG USERNAME=2ACHOLSK1
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install ros-humble-navigation2 -y
RUN apt-get install ros-humble-nav2-bringup -y
RUN apt-get install ros-humble-turtlebot3-gazebo -y
# RUN apt-get install rviz2 - y
RUN apt-get install -y python3-pip

ENV SHELL /bin/bash

RUN apt-get autoremove -y
RUN apt-get clean -y

USER $USERNAME
CMD ["/bin/bash"]