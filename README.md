# ROS Knowledge-Based Robotics
This repository contains an overview over knowledge-based robotics projects in ROS and provides a docker container for testing.

## Build Container

```shell
docker build -t knowledge-based-ros .
```

## Included Projects

### KANT (Knowledge mAnagemeNT) for ROS 2

**Example**
In the login shell run this example in which the python file does not provide any output
```sh
# MongoDB in the background
mongod &
python3 src/kant/kant/kant_dao/kant_dao/example_node.py
# Tear down MongoDB
killall mongod
```

### SWI Prolog
Knowrob is built on SWI Prolog and there is a separate project with ROS 2 binding: https://github.com/SWI-Prolog/rclswi

**Examples**
`rclswi` examples [are in the repository](https://github.com/guillaumeautran/rclswi/tree/galactic-devel/examples).  
They can be executed from the workspace root, so `/colcon_ws`

### KnowRob
> KnowRob is a knowledge processing system that combines knowledge representation and reasoning methods with techniques for acquiring knowledge and for grounding the knowledge in a physical system and can serve as a common semantic framework for integrating information from different sources. KnowRob combines static encyclopedic knowledge, common-sense knowledge, task descriptions, environment models, object information and information about observed actions that has been acquired from various sources (manually axiomatized, derived from observations, or imported from the web). 

[Knowrob](https://knowrob.org/) is built on SWI Prolog. It is a project that can be included into other software and provides python bindings.


### PlanSys2

> ROS2 Planning System (plansys2 in short) is a project whose objective is to provide Robotics developers with a reliable, simple, and efficient PDDL-based planning system. It is implemented in ROS2, applying the latest concepts developed in this currently de-facto standard in Robotics.

**Examples**
A set of examples that come from the project are cloned into `/colcon_ws/src/plansys2_examples`

### KnowledgeCore
> KnowledgeCore is a RDFlib-backed minimalistic knowledge base, initially designed for robots (in particular human-robot interaction or multi-robot interaction).

**Examples**
To start the knowledge base as a server, simply type:
```shell
knowledge_core&
```
Example with `pykb`: https://github.com/severin-lemaignan/pykb

