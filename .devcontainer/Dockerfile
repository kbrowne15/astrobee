FROM ghcr.io/nasa/astrobee:latest-rolling-ubuntu20.04
ENV DEBIAN_FRONTEND=dialog

RUN mkdir /src && mkdir /src/astrobee && apt-get update && apt-get install ros-rolling-domain-bridge && apt-get install vim
