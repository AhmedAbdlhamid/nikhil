# Traffic Sign Reconition

Assumption: the BIMI lab folder layout.

## How to Use

### Preparation Environments

Create two environments.

```
$ cd ~/bimi/robotics/av/envs/
$ conda env create -f ros.yaml
$ conda env create -f neural_net.yaml
```

## Folder layout
```
- traffic_signs: your home folder for this project
  - catkin_ws: ROS workspace
  - darknet: Neural network for object detection using Yolo-v3
  - neural_net: Deep neural network module for training and testing your car
  - weights: trained network parameters (.h5) and network architecture (.json)
```

## Contributions

This project would not be possible without many contributors including Jaerock Kwon, Ahmed, Nikhil, Ninad, and Balu with many generous open source projects including car_demo (osrf), CAT Vehicle, dbw_mkz, etc. 
 
## License

CC0-1.0.

