FROM ros:melodic

# --- Usuario no root "ros" ---
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo git \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# --- Herramientas de compilación y visualización ---
RUN apt-get update && apt-get install -y \
        python-catkin-tools \
        python-rosdep \
        build-essential \
        nano \
        ros-melodic-rviz \
    && rm -rf /var/lib/apt/lists/*

# --- Script de entrada ---
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER $USERNAME
WORKDIR /home/$USERNAME

RUN mkdir -p /home/$USERNAME/catkin_ws/src

# --- Cargar el entorno de ROS automáticamente al abrir terminal ---
RUN echo "source /opt/ros/melodic/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "[ -f /home/$USERNAME/catkin_ws/devel/setup.bash ] && source /home/$USERNAME/catkin_ws/devel/setup.bash" >> /home/$USERNAME/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
