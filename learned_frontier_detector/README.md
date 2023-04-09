# learning based frontier detection

Dataset annotated via Roboflow and trained with Google Colab.

More details will be added soon.

## Resources

- [Transfer learning](https://kikaben.com/yolov5-transfer-learning-dogs-cats/) - The key take away is to freeze the backbone to only update the head layers during training. This is beneficial when the data set is small and you want to benefit from the pre-trained backbone. To determine which layers to freeze look at the model structure to determine the number of layers.

- [Yolov5 training tips](https://github.com/ultralytics/yolov5/wiki/Tips-for-Best-Training-Results)