<<<<<<< HEAD
README
=======
# Ubuntu-Based CAN Communication Robot Drummer
This project is a unique endeavor that leverages CAN communication to control nine motors within a Linux Ubuntu environment, enabling a robot to play drums. It's designed for music enthusiasts and tech aficionados alike, blending robotics with artistry to create rhythmic music.


## Features
- **Precision Motor Control**: Utilizing advanced algorithms in C++ to control Tmotor and Maxon motors with high precision, enabling accurate and dynamic drumming.
- **Real-Time System Performance**: Leveraging Linux Ubuntu with a Real-Time Kernel to ensure immediate response and consistent performance during drumming sequences.
- **Integrated User Interface**: A collaborative effort in Qt for GUI development, focusing on seamless integration between the control logic and user interface for intuitive user interaction.
- **Efficient Motor Communication**: Implementing the CAN Bus Protocol with SocketCAN Library for reliable and efficient communication between the control system and motors.
- **Robust Sensory Feedback**: Incorporating USB IO devices and proximity sensors for precise homing and positioning, enhancing the overall accuracy and consistency of the robot's movements.



## Technology Stack

### Software
- **C++**: Used for core algorithm development and motor control logic.
- **Linux Ubuntu with Real-Time Kernel**: Operating system platform modified for robust and stable performance with real-time capabilities.
- **Qt (GUI Development)**: Collaboratively developed with another developer, focusing on integrating the control logic with the GUI for enhanced user interaction.
- **SocketCAN Library**: Employed for implementing the CAN Bus Protocol, facilitating efficient communication with the motors.
- **Multi-Threading**: Implemented throughout the system to manage concurrent processes effectively. This approach is crucial for real-time data processing, ensuring synchronized motor control and maintaining system stability under various operational conditions.

### Hardware
- **Tmotor AK10-9 and AK70-10 Motors**: High-performance motors used for precise and powerful drumming actions.
- **Maxon Motor (EPOS4 CAN compact 50/15)**: Integrated for precise motor control, contributing to the accuracy and consistency of the drumming robot.
- **USB IO Device**: Used exclusively for connecting and interfacing with sensors.
- **Proximity Sensors**: Employed for homing purposes, ensuring accurate positioning and movement of the robot.
- **Peak USBtoCAN Device**: Used to interface the system with CAN networks, ensuring reliable communication with motor controllers.



## Screenshots and Demo
Include some screenshots of your project in action, and/or links to demo videos.

<img src="./image/Ready_Pos.jpg" width="750" alt="Screenshot Front Robot">
<img src="./image/Ready_Pos2.jpg" width="750" alt="Screenshot Ready Position diagonal Robot">
<img src="./image/Qt.png" width="750" alt="Screenshot for Qt GUI">

### 🎥 **Check out the drum robot video playlist here:** [**Drum Robot Video Playlist Link**](https://www.youtube.com/playlist?list=PLZbG63B7tdmLVjM_1uWHxAeesx-kkmfXY)




## License
State the license under which the project is available. Example: This project is licensed under the MIT License - see the LICENSE.md file for details.
>>>>>>> feature : Reinitialized repository
