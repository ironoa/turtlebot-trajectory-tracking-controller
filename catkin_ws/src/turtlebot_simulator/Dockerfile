FROM ros:noetic-ros-core

# rqt
RUN apt -y update && apt -y install ros-noetic-rqt ros-noetic-rqt-common-plugins

RUN apt -y update && apt -y install tmux nano vim curl gnupg iputils-ping
RUN apt -y update && apt -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Install Qt5 development libraries (to support GUI packages like turtlesim)
RUN apt -y update && apt -y install qt5-default qtbase5-dev qtbase5-dev-tools

RUN rosdep init && rosdep update

RUN apt -y install python3-pip
RUN pip3 install --upgrade "numpy>=1.19.5,<1.27.0" 
RUN pip3 install matplotlib control

WORKDIR /catkin_ws

RUN echo "source /opt/ros/noetic/setup.bash" | tee -a /etc/bash.bashrc /etc/profile
RUN echo "source /catkin_ws/devel/setup.bash" | tee -a /etc/bash.bashrc /etc/profile

CMD /bin/bash

