# The RAPTOR framework

This repository is at the core of the RAPTOR framework. It provides abstractions for different objects from a motion capture system such as a drone or other objects (here, *participants*) and core functionality, particularly for the quadcopter. This allows for writing very compact code in a [reference generator](https://github.com/raptor-ethz/reference_generator) to control the drone and other components of the RAPTOR system without having to reimplement basic functions every time. 

## Installation

This repository will be included in all of our other repositories wherever it is needed. However, for development, you may want to have this repository cloned separately. 

You will need the following dependencies installed on your system:
- [eProsima FastDDS](https://fast-dds.docs.eprosima.com/en/latest/installation/binaries/binaries_linux.html)
- [MAVSDK](https://mavsdk.mavlink.io/main/en/cpp/guide/installation.html)
- yaml-cpp: This can be installed with `sudo apt install libyaml-cpp-dev` on Ubuntu and `brew install yaml-cpp` on macOS
  
After the dependencies have been installed, clone this repository using: 

```bash
git clone --recursive https://github.com/raptor-ethz/raptor_lib
```

## Functionality

### Participants

We provide abstractions for a number of relevant items for aerial grasping and implementations for participants that are rich in functionality like the quadcopter or the gripper attached to the quadcopter which can be used in a reference generator. Actions may require different components running, e.g. the swoop action for the quadcopter requires you to have a gripper on the drone and a [gripper interface](https://github.com/raptor-ethz/gripper_interface) running on an onboard computer.  

### Logging

There is a built in logging functionality which can also be used in a reference generator to acquire and store information about different objects. 

### Trajectories

There is code for trajectory generation, which is utilized in the implementation of certain quadcopter actions. 

