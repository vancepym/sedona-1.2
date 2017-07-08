#!/bin/bash
# Credits
set -e -x -u

BASE_SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

IMAGE_DISTRO=centos
[ $# -gt 0 ] && [ x$1 != x ] && [ -d ${BASE_SCRIPT_DIR}/$1 ] && IMAGE_DISTRO=$1

export DOCKER_ENV_VERSION="20170709"
export BASE_IMAGE_NAME="sedona/${IMAGE_DISTRO}-base:${DOCKER_ENV_VERSION}"

docker build --rm=true -t ${BASE_IMAGE_NAME} ${BASE_SCRIPT_DIR}/${IMAGE_DISTRO}
