FROM ros:melodic-robot


ARG SONIA_USER=sonia
ARG SONIA_UID=50000
ARG MODULE_NAME=provider_kill_mission

ENV SONIA_HOME=/home/${SONIA_USER}
ENV SONIA_WS=${SONIA_HOME}/ros_sonia_ws/
ENV SONIA_USER=${SONIA_USER}
ENV SONIA_UID=${SONIA_UID}
ENV MODULE_NAME=${MODULE_NAME}
ENV MODULE_PATH=${SONIA_WS}/${MODULE_NAME}
ENV LAUNCH_FILE=${MODULE_NAME}.launch
ENV ENTRYPOINT_FILE=sonia_entrypoint.sh

ENV ROS_WS_SETUP=/opt/ros/${ROS_DISTRO}/setup.bash
ENV SONIA_WS_SETUP=/test/devel/setup.bash

RUN useradd --uid ${SONIA_UID} --create-home ${SONIA_USER}

USER ${SONIA_USER}
WORKDIR ${SONIA_HOME}

# COPY ./launch/${LAUNCH_FILE} ${LAUNCH_FILE}
# COPY ./script/${ENTRYPOINT_FILE} /${ENTRYPOINT_FILE}



RUN mkdir test
WORKDIR test
RUN mkdir src


# COPY pfe_sandbox_nodelet1 src/pfe_sandbox_nodelet1
# COPY pfe_sandbox_nodelet2 src/pfe_sandbox_nodelet2
COPY launching_file.launch launching_file.launch
COPY sonia_entrypoint.sh sonia_entrypoint.sh
RUN /bin/bash -c "source ${ROS_WS_SETUP}; catkin_make"
ENTRYPOINT ["/test/sonia_entrypoint.sh"]
CMD ["roslaunch", "launching_file.launch"]
