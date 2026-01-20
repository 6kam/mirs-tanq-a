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
    libcanberra-gtk3-module \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/projects/mirsws

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo 'export PS1="\[\e[32m\]\u@mirs-sim\[\e[m\]:\[\e[34m\]\w\[\e[m\]\\$ "' >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias cbs='colcon build --symlink-install --packages-select'" >> /root/.bashrc && \
    echo "alias cbt='colcon build --symlink-install --packages-up-to'" >> /root/.bashrc

CMD ["bash"]