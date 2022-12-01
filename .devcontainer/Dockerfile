FROM ghcr.io/nasa/astrobee:latest-rolling-ubuntu20.04
ENV DEBIAN_FRONTEND=dialog

RUN mkdir /src && mkdir /src/astrobee && mkdir /src/is-workspace && git clone https://github.com/eProsima/Integration-Service.git /src/is-workspace/src/Integration-Service --recursive && git clone https://github.com/eProsima/ROS2-SH.git /src/is-workspace/src/ROS2-SH && apt install libyaml-cpp-dev
