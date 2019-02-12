# ros2_speech_recognition

Assuming your ROS2 installation is in `/opt/ros/bouncy`

```
$ pip3 install --user SpeechRecognition

$ source /opt/ros/bouncy/setup.bash

# build roboy_communication
$ mkdir ros2_ws/src -p
$ cd ros2_ws/src
$ git clone https://github.com/Roboy/roboy_communication.git -b bouncy
$ cd ..
$ colcon build

$ source install/setup.bash

# start the endless recognition loop (listen till pause <-> recognize)
$ cd ros2_ws/src
$ git clone https://github.com/Roboy/ros2_speech_recognition.git
$ cd ros2_speech_recognition
$ python3 recognition_node.py

# in a separate terminal you can see the published result
$ source ros2_ws/install/setup.bash
$ ros2 topic echo /roboy/cognition/speech/recognition 
source: 0
text: So it's very difficult or not.

source: 1
text: No.
```
