ARG BASE_IMAGE="docker.pkg.github.com/sonia-auv/sonia_messages/sonia_messages:x86-robot-latest"

FROM ${BASE_IMAGE}

USER root

ARG BUILD_DATE
ARG VERSION

ENV NODE_NAME=provider_kill_mission

LABEL net.etsmtl.sonia-auv.node.build-date=${BUILD_DATE}
LABEL net.etsmtl.sonia-auv.node.version=${VERSION}
LABEL net.etsmtl.sonia-auv.node.name=${NODE_NAME}


ENV SONIA_WS=${SONIA_HOME}/ros_sonia_ws

ENV NODE_NAME=${NODE_NAME}
ENV NODE_PATH=${SONIA_WS}/src/${NODE_NAME}
ENV LAUNCH_FILE=${NODE_NAME}.launch
ENV SCRIPT_DIR=${SONIA_WS}/scripts
ENV ENTRYPOINT_FILE=sonia_entrypoint.sh
ENV LAUNCH_ABSPATH=${NODE_PATH}/launch/${LAUNCH_FILE}
ENV ENTRYPOINT_ABSPATH=${SCRIPT_DIR}/${ENTRYPOINT_FILE}

ENV SONIA_WS_SETUP=${SONIA_WS}/devel/setup.bash

WORKDIR ${SONIA_WS}

COPY . ${NODE_PATH}
RUN bash -c "source ${ROS_WS_SETUP}; source ${SONIA_WS_SETUP}; source ${BASE_LIB_WS_SETUP}; catkin_make"

RUN chown -R ${SONIA_USER}: ${SONIA_WS}
USER ${SONIA_USER}

RUN mkdir ${SCRIPT_DIR}
RUN echo "$ENTRYPOINT_ABSPATH" > ${SCRIPT_DIR}/entrypoint.sh
RUN echo "roslaunch $LAUNCH_ABSPATH" > ${SCRIPT_DIR}/launch.sh

ENTRYPOINT ["/bin/bash", "./scripts/entrypoint.sh"]
CMD ["bin/bash", "./scripts/launch.sh"]
