# learning based frontier detection

This is a ROS 2 package that consumes an occupancy grid map and finds frontiers. It acts as a server to any node that requests a frontier goal pose.

## Setup

```
pip install -r requirements.txt

colcon build
```

## Run

```
ros2 run learned_frontier_detector frontier_detector
```

## Training
Roboflow was used to label the dataset. Then a google colab notebook was used to perfrom transfer learning with the custom dataset.

See [yolov5-custom-training-64.ipynb](learned_frontier_detector/training/yolov5-custom-training-64.ipynb) for the commands to train the network. The backbone of the network was frozen to finetune the head of the network.

## Resources

- [Transfer learning](https://kikaben.com/yolov5-transfer-learning-dogs-cats/) - The key take away is to freeze the backbone to only update the head layers during training. This is beneficial when the data set is small and you want to benefit from the pre-trained backbone. To determine which layers to freeze look at the model structure to determine the number of layers.

- [Yolov5 training tips](https://github.com/ultralytics/yolov5/wiki/Tips-for-Best-Training-Results)