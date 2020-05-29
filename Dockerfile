FROM docker.pkg.github.com/sonia-auv/sonia_msgs/sonia_msgs_robot:1.0.7

USER root

ARG BUILD_DATE
ARG VERSION
ENV SONIA_USER=sonia
ENV SONIA_UID=50000
ENV MODULE_NAME=provider_kill_mission

LABEL maintainer="club.sonia@etsmtl.net"
LABEL net.etsmtl.sonia-auv.build-date=${BUILD_DATE}
LABEL net.etsmtl.sonia-auv.version=${VERSION}
LABEL net.etsmtl.sonia-auv.name=${MODULE_NAME}


ENV SONIA_HOME=/home/${SONIA_USER}
ENV SONIA_WS=${SONIA_HOME}/ros_sonia_ws
ENV SONIA_USER=${SONIA_USER}
ENV SONIA_UID=${SONIA_UID}
ENV MODULE_NAME=${MODULE_NAME}
ENV MODULE_PATH=${SONIA_WS}/${MODULE_NAME}
ENV LAUNCH_FILE=${MODULE_NAME}.launch
ENV ENTRYPOINT_FILE=sonia_entrypoint.sh
ENV LAUNCH_ABSPATH=${MODULE_PATH}/launch/${LAUNCH_FILE}
ENV ENTRYPOINT_ABSPATH=${MODULE_PATH}/script/${ENTRYPOINT_FILE}

ENV ROS_WS_SETUP=/opt/ros/${ROS_DISTRO}/setup.bash
ENV SONIA_WS_SETUP=${SONIA_HOME}/ros_sonia_ws/devel/setup.bash

RUN bash -c 'mkdir -p ${SONIA_WS}/{launch,msg,script,src,srv}'

WORKDIR ${SONIA_WS}

COPY . ${MODULE_PATH}
RUN bash -c "source ${ROS_WS_SETUP}; catkin_make"

RUN chown -R ${SONIA_USER}: ${SONIA_WS}
USER ${SONIA_USER}

RUN echo "$ENTRYPOINT_ABSPATH" > ./entrypoint.sh
RUN echo "roslaunch $LAUNCH_ABSPATH" > ./launch.sh

ENTRYPOINT ["/bin/bash", "./entrypoint.sh"]
CMD ["bin/bash", "./launch.sh"]
