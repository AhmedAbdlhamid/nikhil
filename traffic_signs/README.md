# Traffic Signs Recognition using YOLO-v3

## Assumptions

- `ros` and `neural_net` environments are already created.
- `ros-$ROS_DISTRO-joystick-drivers` is installed.

```
$ sudo apt install ros-$ROS_DISTRO-joystick-drivers
```
### Setup Logitech G920
```
$ cd catkin_ws
$ bash src/car_demo/car_demo/nodes/setup_driving_simulator.sh
```

## How to build

Start `ros` environmnet.
```
$ conda activate ros
```

Go to `catkin_ws` and build it.
```
(ros) $ cd catkin_ws
(ros) $ catkin_make
```

## How to start 

If you have not activated `ros` environment, do so with the following command.
```
$ conda activate ros
```

First enable your workspace environment.
```
(ros) $ . devel/setup.bash
```
Then, you will be able to start *rviz* and *Gazebo*.
```
(ros) $ roslaunch car_demo test_track.launch
```

## How to collect data

If you have not activated `ros` environment, do so with the following command.
```
$ conda activate ros
```

Open a new terminal and run the following commands.
```
(ros) $ . devel/setup.bash
(ros) $ cd catkin_ws
(ros) $ rosrun data_collection data_collection.py your_data_name
```
Your data will be saved at `data/your_data_name/year_month_date_time/*.jpg`. All image file names with corresponding steering angle and throttle value will be saved in the same folder.

## How to train

Activate `neural_net` environment.

```
$ conda activate neural_net
```

Go to `neural_net` folder.

```
(neural_net) $ python train.py ../data/your_data_name/year_month_date_time/
```

After the training is done, you will have .h5 and .json file in the `../data/your_data_name/` folder.

## How to start darknet_ros

```
(neural_net) $ cd catkin_ws
(neural_net) $ roslaunch darknet_ros traffic_signs.launch
```

This will start rviz and Gazebo with the Yolo window to show object detection results.

## How to run the trained ANN controller

Activate `neural_net` environment if you haven't yet.

```
$ conda activate neural_net
```

Assuming the `traffic_signs.launch`in the `darknet_ros` package is already started, you can run the trained artificial neural network with following commands.

```
(neural_net) $ cd catkin_ws
(neural_net) $ rosrun run_neural_darknet run_neural_darknet.py ../data/your_data_name/year_month_date_time_n0
```
Then it will load the trained weight `your_data_name/year_month_date_time_n0.h5` and run it to generate the steering angle.
