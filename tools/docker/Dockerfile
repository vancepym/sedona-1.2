#
# Copyright 2015 XXX. All Rights Reserved.
# Licensed under the Academic Free License version 3.0
#
# History:
#    2015 Oct 15   Divisuals   Initial Docker Image
#    2016 Feb 23   Divisuals   Updated to latest JDK 8_73
#    2017 Apr 24   Scott Fan   Updated to latest JDK 8_131
#

FROM toopher/centos-i386:centos6
MAINTAINER Divisuals <divisuals.net@gmail.com>

LABEL Vendor="Sedona"
LABEL version=develop

ENV JAVA_VERSION=8 \
    JAVA_UPDATE=131 \
    JAVA_BUILD=11 \
    JAVA_PATH=d54c1d3a095b4ff2b6607d096fa80163

ENV JAVA_HOME /jdk1.${JAVA_VERSION}.0_${JAVA_UPDATE}

RUN	yum install -y wget which tar git gcc glibc-static python-argparse.noarch \
	&& wget --header "Cookie: oraclelicense=accept-securebackup-cookie;" "http://download.oracle.com/otn-pub/java/jdk/${JAVA_VERSION}u${JAVA_UPDATE}-b${JAVA_BUILD}/${JAVA_PATH}/jdk-${JAVA_VERSION}u${JAVA_UPDATE}-linux-i586.tar.gz" \
	&& tar xf jdk-${JAVA_VERSION}u${JAVA_UPDATE}-linux-i586.tar.gz \
	&& rm -f /jdk-${JAVA_VERSION}u${JAVA_UPDATE}-linux-i586.tar.gz \
	&& rm -rf $JAVA_HOME/*src.zip \
						$JAVA_HOME/lib/missioncontrol \
						$JAVA_HOME/lib/visualvm \
						$JAVA_HOME/lib/*javafx* \
						$JAVA_HOME/jre/lib/plugin.jar \
						$JAVA_HOME/jre/lib/ext/jfxrt.jar \
						$JAVA_HOME/jre/bin/javaws \
						$JAVA_HOME/jre/lib/javaws.jar \
						$JAVA_HOME/jre/lib/desktop \
						$JAVA_HOME/jre/plugin \
						$JAVA_HOME/jre/lib/deploy* \
						$JAVA_HOME/jre/lib/*javafx* \
						$JAVA_HOME/jre/lib/*jfx* \
						$JAVA_HOME/jre/lib/amd64/libdecora_sse.so \
						$JAVA_HOME/jre/lib/amd64/libprism_*.so \
						$JAVA_HOME/jre/lib/amd64/libfxplugins.so \
						$JAVA_HOME/jre/lib/amd64/libglass.so \
						$JAVA_HOME/jre/lib/amd64/libgstreamer-lite.so \
						$JAVA_HOME/jre/lib/amd64/libjavafx*.so \
						$JAVA_HOME/jre/lib/amd64/libjfx*.so \
	&& rm -rf /usr/share/locale/* \
	&& yum clean all

ENV PATH $PATH:$JAVA_HOME/bin
