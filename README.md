# ROS2 Humble Development Environment

This repository contains a Docker-based development environment for ROS2 Humble.

## Features

- ROS2 Humble
- NVIDIA GPU support
- Host network access
- USB and video device access
- Development tools (git, vim, zsh)
- Oh My Zsh pre-installed
- Automatic user creation with sudo access
- X11 forwarding for GUI applications

## Prerequisites

- Docker
- Docker Compose
- NVIDIA Container Toolkit

## Directory Structure

```markdown
.
├── docker/
│   ├── Dockerfile
│   └── .env
├── docker-compose.yml
├── README.md
└── src/                    # Mount your ROS2 packages here
```

## Usage

1. Build the container:

   ```bash
   docker-compose build
   ```

2. Start the container:

   ```bash
   docker-compose up -d
   ```

3. Enter the container:

   ```bash
   docker-compose exec ros2-dev zsh
   ```

4. Stop the container:

   ```bash
   docker-compose down
   ```

## Development

Your ROS2 packages should be placed in the `src/` directory. This directory is mounted inside the container at `/home/host_user/ros2_ws/src/`.

## Notes

- The container runs with your host user's UID to avoid permission issues
- GUI applications should work out of the box with X11 forwarding
- USB devices and video devices are accessible inside the container
- The container runs with host network mode for easy networking