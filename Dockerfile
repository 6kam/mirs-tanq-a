FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    build-essential \
    cmake \
    git \
    vim \
    usbutils \
    x11-apps \
    libcanberra-gtk-module \
    libcanberra-gtk3-module 

WORKDIR /root/projects/mirsws
ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo 'export PS1="\[\e[32m\]\u@mirs-sim\[\e[m\]:\[\e[34m\]\w\[\e[m\]\\$ "' >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias cbs='colcon build --symlink-install --packages-select'" >> /root/.bashrc && \
    echo "alias cbt='colcon build --symlink-install --packages-up-to'" >> /root/.bashrc && \
    echo "alias ru='rosdep update'" >> /root/.bashrc && \
    echo "alias ri='rosdep install --from-path src --ignore-src -r -y'" >> /root/.bashrc && \
    echo "alias si='source install/setup.bash'" >> /root/.bashrc

CMD ["bash"]