FROM ros:humble-ros-base

RUN mkdir -p /colcon_ws/src
ENV WS=/colcon_ws
ENV SRC=$WS/src
WORKDIR $SRC
RUN apt update && apt install -y python3-pip curl git wget gpg psmisc software-properties-common

### Potassco
RUN add-apt-repository ppa:potassco/stable -y
RUN apt update && apt install -y clingo
RUN pip install clingo

### Knowrob
RUN apt install -y libboost-dev libboost-program-options-dev libboost-serialization-dev libboost-python-dev
RUN sudo apt-add-repository -y ppa:swi-prolog/stable && sudo apt update && sudo apt install -y swi-prolog libraptor2-dev libmongoc-dev librdf0-dev
WORKDIR ${WS}
RUN git clone https://github.com/knowrob/knowrob && mkdir -p knowrob/build && touch knowrob/COLCON_IGNORE
WORKDIR ${WS}/knowrob/build
RUN cmake -DCATKIN=OFF -DPYTHON_MODULE_LIBDIR="dist-packages" ..
RUN make -j8 && make install
WORKDIR $SRC

## SWI-Prolog client for ROS2
RUN apt install -y ros-humble-turtlesim
# Fork with adaption to ROS2 Galactic of main repo (https://github.com/SWI-Prolog/rclswi)
RUN git clone --branch galactic-devel https://github.com/guillaumeautran/rclswi/

### PlanSys2
RUN sudo apt install -y ros-humble-plansys2-* ros-humble-test-msgs
RUN git clone -b humble https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples plansys2_examples

### KnowledgeCore
RUN pip3 install rdflib reasonable pykb && mkdir -p $SRC/knowledge_core
WORKDIR $SRC/knowledge_core
RUN git clone https://github.com/pal-robotics/kb_msgs
RUN git clone https://github.com/severin-lemaignan/knowledge_core.git
WORKDIR $SRC/knowledge_core/knowledge_core
RUN python3 setup.py install && touch COLCON_IGNORE
RUN echo 'export PATH=$PATH:/usr/lib/knowledge_core' >> ~/.bashrc
WORKDIR $SRC

### Ontologenius
RUN apt install -y ros-cmake-modules ros-humble-libcurl-vendor qtbase5-dev libopencv-dev
RUN git clone https://github.com/sarthou/ontologenius

### SkiROS2
RUN mkdir SkiROS2
WORKDIR $SRC/SkiROS2
RUN git clone --branch ros2 https://github.com/RobotLabLTH/SkiROS2
RUN git clone --branch ros2 https://github.com/RobotLabLTH/skiros2_std_lib
WORKDIR $SRC

### MERLIN 2
## KANT is installed as a submodule in the MERLIN 2 repository
RUN git clone --recurse-submodules https://github.com/MERLIN2-ARCH/merlin2.git
WORKDIR $SRC/merlin2
# SMTPlan+, sst, mongo dependencies
RUN apt install -y libboost-dev libpulse-dev libz3-dev python3-dev libportaudio2 libportaudiocpp0 portaudio19-dev libasound-dev swig
# unified-planning
RUN pip3 install --pre unified-planning[pyperplan,tamer]
# Install MongoDB
RUN apt install -y libboost-dev
RUN pip3 install mongoengine dnspython && apt install libmongoc-dev libmongoc-1.0-0 -y
RUN wget -qO - https://www.mongodb.org/static/pgp/server-6.0.asc | sudo gpg --dearmor -o /usr/share/keyrings/mongodb-server.gpg
RUN echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server.gpg] https://repo.mongodb.org/apt/ubuntu jammy/mongodb-org/6.0 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-6.0.list
RUN apt update && apt install -y mongodb-org
RUN mkdir -p /data/db && chown -R mongodb:mongodb /data/db

RUN ./scrips/install_mongocxx.sh
# tts
RUN apt install -y espeak speech-dispatcher festival festival-doc festvox-kdlpc16k festvox-ellpc11k festvox-italp16k festvox-itapc16k mpg321
# Merlin
RUN pip3 install -r requirements.txt
RUN python3 merlin2_arch/merlin2_reactive_layer/speech_to_text/nltk_download.py
WORKDIR $SRC

### Build the workspace
WORKDIR $WS
# Install dependencies. Skipping 'python3-reasonable' comes from KnowledgeCore
RUN rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y --skip-keys python3-reasonable
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install
RUN echo "source $WS/install/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/bin/bash"]