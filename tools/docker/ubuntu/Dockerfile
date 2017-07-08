FROM phusion/baseimage
MAINTAINER Scott Fan <fancp2007@gmail.com>

LABEL vendor="sedona" version=develop

ENV TIMEZONE="Asia/Shanghai"

RUN echo 'APT::Install-Recommends 0;' >> /etc/apt/apt.conf.d/01norecommends \
 && echo 'APT::Install-Suggests 0;' >> /etc/apt/apt.conf.d/01norecommends \
 && apt-get update && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y \
 && rm -fv /etc/localtime && ln -sfv /usr/share/zoneinfo/${TIMEZONE} /etc/localtime \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata

RUN DEBIAN_FRONTEND=noninteractive apt-get -y install python gcc gcc-multilib openjdk-8-jdk-headless \
 && apt-get -y clean autoclean \
 && apt-get -y autoremove \
 && rm -rfv /var/lib/apt/lists/*

ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64
ENV PATH $PATH:$JAVA_HOME/bin
