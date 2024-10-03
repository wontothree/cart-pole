# Use the official Ubuntu base image
FROM ubuntu:jammy

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    gcc-arm-none-eabi \
    build-essential \
    libnewlib-arm-none-eabi \
    git \
    sudo \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set up a non-root user
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to the non-root user
USER $USERNAME

# Clone and set up Pico SDK
RUN sudo mkdir -p /usr/local/pico-sdk \
    && sudo chown $USERNAME:$USERNAME /usr/local/pico-sdk \
    && git clone https://github.com/raspberrypi/pico-sdk.git --branch master /usr/local/pico-sdk \
    && cd /usr/local/pico-sdk \
    && git submodule update --init

# Set environment variables
ENV PICO_SDK_PATH="/usr/local/pico-sdk"

# The CMD instruction should be in the docker-compose.yml or when running the container
CMD ["/bin/bash"]
