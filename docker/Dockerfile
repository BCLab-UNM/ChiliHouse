FROM osrf/ros:kinetic-desktop-xenial

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kinetic-desktop-full=1.3.2-0* \
    python-rosinstall python-catkin-tools libusb-dev ros-kinetic-robot-localization\
    ros-kinetic-hector-gazebo-plugins ros-kinetic-joystick-drivers \
    ros-kinetic-usb-cam ros-kinetic-rosserial-python ros-kinetic-rosserial-arduino \
    ros-kinetic-multimaster-fkie ros-kinetic-grid-map ipython git \
    && rm -rf /var/lib/apt/lists/*
    
#From https://github.com/BCLab-UNM/Swarmathon-Docs/blob/master/PreinstalledCompetitionPackages.md
#ros-kinetic-desktop-full python-rosinstall python-catkin-tools ros-kinetic-robot-localization ros-kinetic-hector-gazebo-plugins ros-kinetic-joystick-drivers git qtcreator libcap2-bin ros-kinetic-grid-map ros-kinetic-rosserial-python ros-kinetic-rosserial-arduino ros-kinetic-usb-cam ros-kinetic-multimaster-fkie ros-kinetic-rosbridge ros-kinetic-rosbridge-server
RUN /bin/bash -c "mkdir -p /ChiliHouse/src"

COPY . /ChiliHouse
RUN chmod 777 /ChiliHouse

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd /ChiliHouse; ls -la; catkin clean; catkin init; catkin build; source ./devel/setup.bash; /ChiliHouse/run.sh"

