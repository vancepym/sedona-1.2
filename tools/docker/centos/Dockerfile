FROM toopher/centos-i386:centos6
MAINTAINER Scott Fan <fancp2007@gmail.com>

LABEL vendor="sedona" version=develop

ENV TIMEZONE="Asia/Shanghai"

RUN yum update -y \
 && rm -fv /etc/localtime && ln -sfv /usr/share/zoneinfo/${TIMEZONE} /etc/localtime \
 && yum install -y tzdata

RUN yum install -y which gcc glibc-static python-argparse.noarch java-1.8.0-openjdk-devel \
 && yum clean all \
 && rm -rfv /usr/share/locale/*

ENV JAVA_HOME /usr/lib/jvm/java-1.8.0-openjdk.i386
ENV PATH $PATH:$JAVA_HOME/bin
