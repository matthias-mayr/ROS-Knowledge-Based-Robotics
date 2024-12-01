FROM ros:humble-ros-base

RUN mkdir -p /colcon_ws/src
ENV WS=/colcon_ws
ENV SRC=$WS/src
WORKDIR $SRC
RUN apt update && apt install -y python3-pip curl git wget gpg psmisc software-properties-common

### KANT
# Adapted instructions from https://github.com/uleroboticsgroup/kant
RUN mkdir -p $SRC/kant
WORKDIR $SRC/kant
# Install MongoDB
RUN apt install -y libboost-dev
RUN pip3 install mongoengine dnspython && apt install libmongoc-dev libmongoc-1.0-0 -y
RUN curl -OL https://github.com/mongodb/mongo-cxx-driver/archive/refs/tags/r3.11.0.tar.gz && tar -xzf r3.11.0.tar.gz
WORKDIR $SRC/kant/mongo-cxx-driver-r3.11.0/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBSONCXX_POLY_USE_BOOST=1 -DBUILD_VERSION=3.11.0
RUN cmake --build . --target install

RUN echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
WORKDIR $SRC/kant
RUN rm r3.11.0.tar.gz && rm -rf mongo-cxx-driver-r3.11.0

RUN wget -qO - https://www.mongodb.org/static/pgp/server-6.0.asc | sudo gpg --dearmor -o /usr/share/keyrings/mongodb-server.gpg
RUN echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server.gpg] https://repo.mongodb.org/apt/ubuntu jammy/mongodb-org/6.0 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-6.0.list
RUN apt update && apt install -y mongodb-org
RUN mkdir -p /data/db && chown -R mongodb:mongodb /data/db

# Clone the repository
RUN git clone https://github.com/uleroboticsgroup/simple_node.git
# Temporary fix for the kant repository until the PR is merged:
RUN git clone --branch fix/humble-jammy https://github.com/matthias-mayr/kant.git



### Ontologenius
RUN apt install -y ros-cmake-modules ros-humble-libcurl-vendor qtbase5-dev libopencv-dev
RUN git clone https://github.com/sarthou/ontologenius


### Build the workspace
WORKDIR $WS
# Install dependencies. Skipping 'python3-reasonable' comes from KnowledgeCore
RUN rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y --skip-keys python3-reasonable
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install
RUN echo "source $WS/install/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/bin/bash"]