FROM nvidia/cuda:11.7.1-base-ubuntu22.04 AS base

# Avoid prompts from apt and setup timezone
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# Setup timezone and locale (avoid prompts)
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Enable BuildKit features
# syntax=docker/dockerfile:1.4

# Setup ROS2 repositories and core dependencies
# Mount apt cache to speed up subsequent builds
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        curl \
        gnupg \
        lsb-release \
        locales \
    && locale-gen en_US.UTF-8 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Second stage with development tools
FROM base AS dev

# Install core build dependencies and dev tools first (changes less frequently)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    vim \
    wget \
    curl \
    sudo

# Setup pip cache
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --upgrade pip

# Install ROS2 and its Python tools (changes more frequently, separate layer)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-desktop \
    ros-humble-ros-base \
    ros-dev-tools

# Install shell tools and productivity utilities (changes less frequently)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    zsh \
    htop \
    tmux \
    unzip \
    zip \
    jq \
    fzf \
    ripgrep \
    fonts-powerline

# Setup user with sudo access
ARG USERNAME=host_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create user and add to sudo group
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to user
USER $USERNAME
WORKDIR /home/$USERNAME

# Create ROS workspace and source directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src

# Copy the entrypoint script
COPY --chmod=755 ros_entrypoint.sh /ros_entrypoint.sh

# Setup ROS environment for both bash and zsh
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc \
    && echo "if [ -f /home/$USERNAME/ros2_ws/install/setup.bash ]; then source /home/$USERNAME/ros2_ws/install/setup.bash; fi" >> ~/.bashrc \
    && echo "if [ -f /home/$USERNAME/ros2_ws/install/setup.zsh ]; then source /home/$USERNAME/ros2_ws/install/setup.zsh; fi" >> ~/.zshrc

# Set environment variables for ROS 2
ENV ROS_DISTRO=humble \
    AMENT_PREFIX_PATH=/opt/ros/humble \
    COLCON_PREFIX_PATH=/opt/ros/humble \
    LD_LIBRARY_PATH=/opt/ros/humble/lib \
    PATH=/opt/ros/humble/bin:$PATH \
    PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages \
    ROS_PYTHON_VERSION=3 \
    ROS_VERSION=2

# Install Oh My Zsh and useful plugins/themes for the unprivileged user
ENV ZSH_CUSTOM=/home/$USERNAME/.oh-my-zsh/custom
RUN git clone --depth=1 https://github.com/ohmyzsh/ohmyzsh.git /home/$USERNAME/.oh-my-zsh \
    && cp /home/$USERNAME/.oh-my-zsh/templates/zshrc.zsh-template /home/$USERNAME/.zshrc \
    && sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME="powerlevel10k\/powerlevel10k"/' /home/$USERNAME/.zshrc || true \
    && git clone --depth=1 https://github.com/romkatv/powerlevel10k.git $ZSH_CUSTOM/themes/powerlevel10k \
    && git clone --depth=1 https://github.com/zsh-users/zsh-autosuggestions.git $ZSH_CUSTOM/plugins/zsh-autosuggestions \
    && git clone --depth=1 https://github.com/zsh-users/zsh-syntax-highlighting.git $ZSH_CUSTOM/plugins/zsh-syntax-highlighting \
    && sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/' /home/$USERNAME/.zshrc || true \
    && echo '\n# Colorful LS and useful aliases' >> /home/$USERNAME/.zshrc \
    && echo "alias ls='ls --color=auto'" >> /home/$USERNAME/.zshrc \
    && echo "export TERM=xterm-256color" >> /home/$USERNAME/.zshrc \
    && chown -R $USER_UID:$USER_GID /home/$USERNAME/.oh-my-zsh /home/$USERNAME/.zshrc

# Set the default shell to zsh
SHELL ["/bin/zsh", "-c"]

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

# Default command (can be overridden)
CMD ["zsh"]