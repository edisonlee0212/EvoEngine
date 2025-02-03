FROM nvidia/cuda:12.8.0-devel-ubuntu24.04 as base

ENV DEBIAN_FRONTEND=noninteractive

# Needed to share GPU
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV NVIDIA_VISIBLE_DEVICES=all

#NVidia RTX GPU Required.
ARG CUDA_ARCHITECTURES=120;90;89;87;86;80;75

#Install required libs
RUN apt-get update
RUN apt-get install -y --no-install-recommends clang-14
RUN apt-get install -y --no-install-recommends git
RUN apt-get install -y --no-install-recommends libwayland-dev
RUN apt-get install -y --no-install-recommends libxkbcommon-dev
RUN apt-get install -y --no-install-recommends xorg-dev
RUN apt-get install -y --no-install-recommends wget
RUN apt-get install -y --no-install-recommends python3.12-dev*
RUN apt-get install -y --no-install-recommends sudo
RUN apt-get install -y --no-install-recommends cmake
RUN apt-get install -y --no-install-recommends ninja-build

#Install Vulkan-SDK
RUN wget -qO- https://packages.lunarg.com/lunarg-signing-key-pub.asc | sudo tee /etc/apt/trusted.gpg.d/lunarg.asc
RUN sudo wget -qO /etc/apt/sources.list.d/lunarg-vulkan-noble.list http://packages.lunarg.com/vulkan/lunarg-vulkan-noble.list
RUN sudo apt update
RUN apt install -y vulkan-sdk

#Get EvoEngine
RUN git clone --recurse-submodules https://github.com/edisonlee0212/EvoEngine.git
WORKDIR /EvoEngine
 
#Build EvoEngine
WORKDIR /EvoEngine
RUN bash ./build.sh

#Set entry point
ENTRYPOINT [  "/bin/bash" ]

#Start terminal
RUN bash