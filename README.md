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

