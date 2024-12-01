# ROS Knowledge-Based Robotics
This repository contains an overview over knowledge-based robotics projects in ROS and provides a docker container for testing.

## Build Container

```shell
docker build -t knowledge-based-ros .
```

## Included Projects

### KANT (Knowledge mAnagemeNT) for ROS 2

**Example:**  
In the login shell run this example in which the python file does not provide any output
```sh
# MongoDB in the background
mongod &
python3 src/merlin2/kant/kant_dao/kant_dao/example_node.py
# Tear down MongoDB
killall mongod
```

### SWI Prolog
Knowrob is built on SWI Prolog and there is a separate project with ROS 2 binding: https://github.com/SWI-Prolog/rclswi

**Examples:**  
`rclswi` examples [are in the repository](https://github.com/guillaumeautran/rclswi/tree/galactic-devel/examples).  
They can be executed from the workspace root, so `/colcon_ws`

### KnowRob
> KnowRob is a knowledge processing system that combines knowledge representation and reasoning methods with techniques for acquiring knowledge and for grounding the knowledge in a physical system and can serve as a common semantic framework for integrating information from different sources. KnowRob combines static encyclopedic knowledge, common-sense knowledge, task descriptions, environment models, object information and information about observed actions that has been acquired from various sources (manually axiomatized, derived from observations, or imported from the web). 

[Knowrob](https://knowrob.org/) is built on SWI Prolog. It is a project that can be included into other software and provides python bindings.

### PlanSys2

> ROS2 Planning System (plansys2 in short) is a project whose objective is to provide Robotics developers with a reliable, simple, and efficient PDDL-based planning system. It is implemented in ROS2, applying the latest concepts developed in this currently de-facto standard in Robotics.

**Examples:**  
A set of examples that come from the project are cloned into `/colcon_ws/src/plansys2_examples`

### KnowledgeCore
> KnowledgeCore is a RDFlib-backed minimalistic knowledge base, initially designed for robots (in particular human-robot interaction or multi-robot interaction).

**Examples:**  
To start the knowledge base as a server, simply type:
```shell
knowledge_core&
```
Example with `pykb`: https://github.com/severin-lemaignan/pykb

### Ontologenius

> Ontologenius is a research project aimed at providing what can be seen as a semantic memory for artificial agents. Focusing only on this task, Ontologenius is currently a robust and efficient open-source software that can be deployed in many applications.

**Examples:**  
Ontologenius features a [Python Tutorial](https://sarthou.github.io/ontologenius/python_Tutorials/Tutorials.html) and a [C++ Tutorial](https://sarthou.github.io/ontologenius/cpp_Tutorials/Tutorials.html).

### SkiROS2

> SkiROS2 is a platform to create complex robot behaviors by composing skills - modular software blocks - into behavior trees. SkiROS offers the following features: A world model as a semantic database to manage environmental knowledge. Reasoning capabilities and automatic inference of skill parameters. An integration point for PDDL task planning

**Examples:**  
A very good introduction into SkiROS2 is the Docker container for the ['Hands-On with ROS 2 Deliberation Technologies' at ROSCon 2024](https://github.com/ros-wg-delib/roscon24-workshop).

There is a [dedicated introduction into using SkiROS2 with pyrobosim](https://github.com/matthias-mayr/skiros2_pyrobosim_lib) as part of this workshop that also briefly explains SkiROS2.

### MERLIN 2

> MERLIN 2 (MachinEd Ros pLanINg) is a cognitive architecture for robots based on ROS 2. It is aimed to be used to generate behaviors in robots using PDDL planners and state machines from YASMIN (Yet Another State MachINe) . PDDL can be manageg using KANT (Knowledge mAnagemeNT).

**Examples:**  
* [PDDL example for creating a new action](https://merlin2.readthedocs.io/en/latest/Creating%20New%20Actions.html#)
* [Navigation demos with a simulated robot](https://merlin2.readthedocs.io/en/latest/Demos.html#) - Dependencies not installed in docker