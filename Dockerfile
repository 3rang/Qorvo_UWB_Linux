FROM ubuntu:20.04

WORKDIR /app

ARG DEBIAN_FRONTEND=noninteractive

# APT packages
RUN apt update && apt install -y git ninja-build gperf \
    ccache dfu-util device-tree-compiler wget vim git tree \
    python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
    make gcc libsdl2-dev crossbuild-essential-armhf


# Raspberry Pi toolchains

RUN apt-get install gcc-arm-linux-gnueabihf -y

# Raspberry Pi toolchains (aarch64)

RUN apt-get install gcc-aarch64-linux-gnu -y




RUN mkdir uwb && cd uwb 

RUN git clone https://github.com/3rang/Qorvo_UWB_Linux.git  

CMD ["/bin/bash"]

