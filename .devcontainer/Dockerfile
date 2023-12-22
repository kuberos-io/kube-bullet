FROM mcr.microsoft.com/devcontainers/base:focal

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################
# Install libraries for rendering
RUN DEBIAN_FRONTEND=noninteractive \
    apt-get update \
    && apt install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    mesa-utils \
    libgl1-mesa-glx \
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-common-dev &&\
    rm -rf /var/lib/apt/lists/*

# Install pip and config python
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.8 10 && \
    DEBIAN_FRONTEND=noninteractive \
    apt-get update \
    && apt install -y -qq --no-install-recommends \
    python3-pip

##############################################################################
##                               Pybullet                                   ##
##############################################################################
ADD requirements.txt /tmp

