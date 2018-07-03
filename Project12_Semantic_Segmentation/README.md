## Project: Semantic Segmentation
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview
---
In this project, we label the pixels of road in the [Kitti Road Dataset](http://www.cvlibs.net/datasets/kitti/eval_road.php) using a Fully Convolutional Network (FCN). 

## Approach
---
The FCN architecture used for semantic segmentation in this project is the FCN-8 described in this [paper](https://people.eecs.berkeley.edu/~jonlong/long_shelhamer_fcn.pdf). The FCN-8 architecture uses the [VGG-16](https://arxiv.org/pdf/1409.1556.pdf) image classifier pre-trained on ImageNet. In the FCN-8 architecture, the final fully connected layer in VGG-16 is discarded and is replaced with 1x1 convolution layers with number of convolution kernels equal to the number of classes. Here there are two classes; Class 1: Road, Class 2: No-Road. The segmentation detail in FCN-8 is improved by using two skip connections, first between max pooling layer, pool 4 and the 2x upsampled 1x1 convolution layer, pool 7. 
The second skip connection is between the max pooling layer, pool 3 and the 2x upsampled fused layer between pool 4 and pool 7. The skip connections have known to capture the finer details of the segmentation. 
Tensorflow API has been used to implement the FCN-8 architecture.     

[//]: # (Image References)

[image1]: ./write_up_images/um_000017.png "Image 1"
[image2]: ./write_up_images/um_000059.png "Image 2"
[image3]: ./write_up_images/um_000083.png "Image 3"
[image4]: ./write_up_images/um_000007.png "Image 4"
[image5]: ./write_up_images/um_000042.png "Image 5"

## Model Training Details:
* Tranposed Convolution Kernel Initializer = Truncated Normal Initializer with Standard Deviation = 0.01
* Tranposed Convolution Kernel Regularizer = Truncated Normal Initializer with Weight = 0.001
* Number of Epochs = 6
* Batch Size = 8
* Learning Rate = 0.0001
* Loss Function = Cross-Entropy
* Optimizer = Adam Optimizer

## Result
---
Using the Kernel initializer and regularizer for the transpose convolution significantly improves the prediction of the road pixels. After around 6 epochs the loss was around 0.02. Increasing the number of epochs beyond led to overfitting of the data. 
Here are some of the sample predictions from the test data set:
![alt text][image1]
![alt text][image2]
![alt text][image3]
![alt text][image4]
![alt text][image5]

In general the trained model performs reasonably well. It appears that the model tends to fail in regions of object shadows for some of the images. This can be improved by augmentation of the data.