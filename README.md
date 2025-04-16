# odrive_interface
This package is used to control an ODrive using ROS2. This package is developed for the Pieperkieker project.

## File tree
```
odrive_interface
├── CMakeLists.txt
├── package.xml
├── README.md
├── arduino
│   └── sketch
│       └── sketch.ino
├── msg
│   ├── OdriveCommand.msg
│   └── OdriveFeedback.msg
└── src
    └── odrive_serial_node.cpp
```

## ROS2 Interface for ODrive
This package provides a simple interface to control an ODrive using ROS2. The interface is designed to be used with an Arduino that is connected to the ODrive via USB. The Arduino acts as a bridge between the ODrive and ROS2, allowing for easy control of the ODrive from a ROS2 node.

## Dependencies
The project has a couple of dependencies. this is split into two parts:
- **Arduino**: 
  - ODriveArduino: https://github.com/odriverobotics/ODriveArduino
  - arduino-CAN: https://github.com/sandeepmistry/arduino-CAN
- **ROS2**:
  - ROS2 humble: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
  - PkgConfig: (sudo apt install pkg-config) https://www.freedesktop.org/wiki/Software/pkg-config/
  - libserialport: (sudo apt install libserialport-dev) https://github.com/sigrokproject/libserialport
 
## Protocol

## Into odrive_interface
The ros2 node has a couple topics it subscribes/pubishes to. these have custom message types. in this chapter these 
topics and message types are described.

### Message types
The custom message types are defined in the `/msg` directory. The message types are as follows:
- **OdriveCommand**: This message is used to send commands to the odrive. The message type is defined as follows:
```
string command_type
float32 value
```
Where:
- `command`: The command to be sent to the odrive. This can be one of the following:
  - `position`: Set the position of the odrive in rotations
  - `velocity`: Send the velocity of the odrive in rps
- `value`: The value to be sent to the node. This is a f32 value.

and finally the `OdriveFeedback` message type is used to send feedback from the odrive. The message type is defined as follows:
```
float32 position
float32 velocity
```
Where:
- `position`: The position of the odrive in rotations. This is a f32 value.
- `velocity`: The velocity of the odrive in rps. This is a f32 value.

### Topics

The ros2 node listens and publishes on the following topics. on these topics custom messages types are used. these can be found in the `/msg` directory.
- **Each node**:
  - /odrive_{node_id}/command: This topic is used to send commands to the odrive. The message type is `OdriveCommand`.
  - /odrive_{node_id}/feedback: This topic is used to receive feedback from the odrive. The message type is `OdriveFeedback`.
- **Combined nodes**
  - /throttle: This topic is used to send a message to all throttle motors this is of type `OdriveCommand`.
  - /steering: This topic is used to send a message to all steering motors this is of type `OdriveCommand`.


## Ros -> Arduino
Ros and arduino communicate with each other via a serial port. This is doe using a simple plaintext protocol.
The protocol is as follows:
```
{P|V} {node_id: int} {value: f32}
```

Where:
- Command type: The type of command to be sent. This can be one of the following:
    - `P`: Set the position of the odrive in rotations
    - `V`: Send the velocity of the odrive in rps
- Node id: The id of the node to be controlled. This is an i32 send as a string.
- Value: The value to be sent to the node. This is a f32 value send as a string.

### Feedback
The odrive will also send feedback to the arduino. This is done using the following protocol:
```
OD{node_id: int}: Pos={position: f32} Vel={velocity: f32}
```

Where:
- `node_id`: The id of the node to be controlled. This is an i32 send as a string.
- `position`: The position of the odrive in rotations. This is a f32 value send as a string.
- `velocity`: The velocity of the odrive in rps. This is a f32 value send as a string.
