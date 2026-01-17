# syntax=docker/dockerfile:1.4
FROM nvidia/cuda:11.7.1-base-ubuntu22.04 AS dev

# Prevent prompts and set locale
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# --- Core dependencies and ROS setup ---
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release locales build-essential cmake git \
        python3-pip vim wget sudo tmux unzip zip jq fzf ripgrep htop bash-completion \
    && locale-gen en_US.UTF-8 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions python3-rosdep python3-vcstool \
        ros-humble-desktop ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# --- Create user with sudo ---
ARG USERNAME=host_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
 && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
 && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

# --- Setup workspace and environment ---
RUN mkdir -p ~/ros2_ws/src

# --- ROS entrypoint ---
COPY --chmod=755 ros_entrypoint.sh /ros_entrypoint.sh

# --- Configure Bash environment ---
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
 && echo "if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi" >> ~/.bashrc \
 && echo "source /usr/share/bash-completion/bash_completion" >> ~/.bashrc \
 && echo "if [ -f /opt/ros/humble/etc/bash_completion.d/ros2-completion.bash ]; then source /opt/ros/humble/etc/bash_completion.d/ros2-completion.bash; fi" >> ~/.bashrc \
 && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> ~/.bashrc

# --- Enable colorful bash prompt and ls ---
RUN echo '# Colorful prompt and ls' >> ~/.bashrc \
 && echo 'export TERM=xterm-256color' >> ~/.bashrc \
 && echo 'alias ls="ls --color=auto"' >> ~/.bashrc \
 && echo 'alias grep="grep --color=auto"' >> ~/.bashrc \
 && echo 'force_color_prompt=yes' >> ~/.bashrc \
 && echo 'if [ -n "$force_color_prompt" ]; then' >> ~/.bashrc \
 && echo '  if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then' >> ~/.bashrc \
 && echo '    color_prompt=yes' >> ~/.bashrc \
 && echo '  fi' >> ~/.bashrc \
 && echo 'fi' >> ~/.bashrc \
 && echo 'if [ "$color_prompt" = yes ]; then' >> ~/.bashrc \
 && echo '  PS1="\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\\$ "' >> ~/.bashrc \
 && echo 'else' >> ~/.bashrc \
 && echo '  PS1="\u@\h:\w\\$ "' >> ~/.bashrc \
 && echo 'fi' >> ~/.bashrc

# --- Environment variables ---
ENV ROS_DISTRO=humble \
    ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3 \
    AMENT_PREFIX_PATH=/opt/ros/humble \
    COLCON_PREFIX_PATH=/opt/ros/humble \
    LD_LIBRARY_PATH=/opt/ros/humble/lib \
    PATH=/opt/ros/humble/bin:$PATH \
    PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
