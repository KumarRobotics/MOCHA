FROM dtcpronto/ros-jazzy:full 

RUN sudo apt update && sudo apt install -y \
 python3-zmq \
 default-jre \
 iputils-ping \
 python3-lz4 \
 python3-defusedxml

RUN cd ws/src && git clone https://github.com/dtc-pronto/dtc-msgs

COPY interface_rajant ws/src/interface_rajant
COPY mocha_core ws/src/mocha_core
COPY mocha_launch ws/src/mocha_launch

RUN /bin/bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    cd /home/dtc/ws && \
    colcon build --symlink-install --cmake-clean-cache"

COPY ./entrypoint.bash entrypoint.bash
RUN sudo chmod +x entrypoint.bash

