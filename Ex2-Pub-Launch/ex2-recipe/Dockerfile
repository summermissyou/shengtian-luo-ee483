# syntax=docker/dockerfile:1.4
# parameters
ARG EXERCISE_NAME="ex2"
ARG DESCRIPTION="Learning Experience about ROS (Robot Operating System)"
ARG MAINTAINER="Romulo Meira-Goes (romulo-goes)"

# ==================================================>
# ==> Do not change the code below this line
ARG ARCH
ARG DISTRO=daffy
ARG DOCKER_REGISTRY=docker.io
ARG BASE_IMAGE=dt-ros-commons
ARG BASE_TAG=${DISTRO}-${ARCH}
ARG LAUNCHER=default

# define base image
FROM ${DOCKER_REGISTRY}/duckietown/${BASE_IMAGE}:${BASE_TAG} as base

# recall all arguments
ARG DISTRO
ARG EXERCISE_NAME
ARG DESCRIPTION
ARG MAINTAINER
ARG BASE_TAG
ARG BASE_IMAGE
ARG LAUNCHER
# - buildkit
ARG TARGETPLATFORM
ARG TARGETOS
ARG TARGETARCH
ARG TARGETVARIANT

# check build arguments
RUN dt-build-env-check "${EXERCISE_NAME}" "${MAINTAINER}" "${DESCRIPTION}"

# define/create repository path
ARG REPO_PATH="${CATKIN_WS_DIR}/src/${EXERCISE_NAME}"
ARG LAUNCH_PATH="${LAUNCH_DIR}/${EXERCISE_NAME}"
RUN mkdir -p "${REPO_PATH}" "${LAUNCH_PATH}"
WORKDIR "${REPO_PATH}"

# keep some arguments as environment variables
ENV DT_MODULE_TYPE="exercise" \
    DT_MODULE_NAME="${EXERCISE_NAME}" \
    DT_MODULE_DESCRIPTION="${DESCRIPTION}" \
    DT_MAINTAINER="${MAINTAINER}" \
    DT_REPO_PATH="${REPO_PATH}" \
    DT_LAUNCH_PATH="${LAUNCH_PATH}" \
    DT_LAUNCHER="${LAUNCHER}"

# install apt dependencies
COPY --from=recipe ./dependencies-apt.txt "${REPO_PATH}/"
RUN dt-apt-install ${REPO_PATH}/dependencies-apt.txt

# install python3 dependencies
ARG PIP_INDEX_URL="https://pypi.org/simple"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
COPY --from=recipe ./dependencies-py3.* "${REPO_PATH}/"
RUN dt-pip3-install "${REPO_PATH}/dependencies-py3.*"

# copy the assets (recipe)
COPY --from=recipe ./assets "${REPO_PATH}/assets"

# copy the source code (recipe)
COPY --from=recipe ./packages "${REPO_PATH}/packages"

# copy the assets (meat)
COPY --from=meat ./assets/. "${REPO_PATH}/assets/"

# copy the source code (meat)
COPY --from=meat ./packages/. "${REPO_PATH}/packages/"


ARG REPO_PATH="${CATKIN_WS_DIR}/src/${EXERCISE_NAME}"
RUN mkdir -p "${REPO_PATH}"
# copy the source code (meat)
COPY --from=meat ./ex_workspace/src "${REPO_PATH}/packages/"

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/

# install launcher scripts
COPY --from=recipe ./launchers/. "${LAUNCH_PATH}/"
RUN dt-install-launchers "${LAUNCH_PATH}"

# define default command
CMD ["bash", "-c", "dt-launcher-${DT_LAUNCHER}"]

# store module metadata
LABEL org.duckietown.label.module.type="exercise" \
    org.duckietown.label.module.name="${EXERCISE_NAME}" \
    org.duckietown.label.module.description="${DESCRIPTION}" \
    org.duckietown.label.platform.os="${TARGETOS}" \
    org.duckietown.label.platform.architecture="${TARGETARCH}" \
    org.duckietown.label.platform.variant="${TARGETVARIANT}" \
    org.duckietown.label.code.location="${REPO_PATH}" \
    org.duckietown.label.code.version.distro="${DISTRO}" \
    org.duckietown.label.base.image="${BASE_IMAGE}" \
    org.duckietown.label.base.tag="${BASE_TAG}" \
    org.duckietown.label.maintainer="${MAINTAINER}"
# <== Do not change the code above this line
# <==================================================
