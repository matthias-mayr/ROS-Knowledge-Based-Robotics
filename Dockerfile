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

### Knowrob
RUN apt install -y libboost-program-options-dev libboost-serialization-dev libboost-python-dev
RUN sudo apt-add-repository -y ppa:swi-prolog/stable && sudo apt update && sudo apt install -y swi-prolog libraptor2-dev libmongoc-dev librdf0-dev
WORKDIR ${WS}
RUN git clone https://github.com/knowrob/knowrob && mkdir -p knowrob/build
WORKDIR ${WS}/knowrob/build
RUN cmake -DCATKIN=OFF -DPYTHON_MODULE_LIBDIR="dist-packages" ..
RUN make -j8 && make install
WORKDIR $SRC

## SWI-Prolog client for ROS2
RUN apt install -y ros-humble-turtlesim
# Fork with adaption to ROS2 Galactic of main repo (https://github.com/SWI-Prolog/rclswi)
RUN git clone --branch galactic-devel https://github.com/guillaumeautran/rclswi/

# Build the workspace

WORKDIR $WS
RUN rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install
RUN echo "source $WS/install/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/bin/bash"]