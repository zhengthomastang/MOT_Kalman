# MOT_Kalman

This repository contains our C++ implementation of (intra-camera) online multiple object tracking based on Kalman filtering. By tuning some hyperparameters, it is capable of reducing false nagatives and false positives. This algorithm is useful for efficient tracklet generation in data association. 

## Introduction

This package is designed for generating 2D tracklets in each single camera. A 2D Kalman-filter-based tracking-by-detection method is adopted. We also provide the capability of reducing false negatives and false positives. Besides, the package includes tools for manual region of interest (ROI) selection. 

## Coding Structure
1. `./obj/` folder: Libraries of .cpp files
2. `./src/` folder: Source code
3. `./data/` folder: Example input files
   1. `cfg.json`: Configuration parameters in JSON format
   2. `vdo.avi`: Input video source
   3. `det.txt`: Detection results in MOTChallenge format

## How to Build
1. Download and make the OpenCV library.
2. Compile using g++ in Linux environment. If you are new to g++ compilation with OpenCV, please refer to this [link](http://answers.opencv.org/question/25642/how-to-compile-basic-opencv-program-in-c-in-ubuntu/). In the command window, you can `cd` to the directory of `./MOT_Kalman-master/` and use the following command to compile our source code, where `bin` is the executable file generated.  
```g++ -I/usr/local/include/ -L/usr/local/lib/ -g -o bin ./src/main.cpp ./src/Cfg.cpp ./src/RoiSel.cpp ./src/ObjDet.cpp ./src/ObjTrk.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lopencv_video -lm```

## How to Use
1. Set the corresponding input/output paths in the configuration file if necessary. **Please note that the input video source (folder of images or video file) is ONLY necessary for plotting tracking results. The ONLY required input for the tracking algorithm is the detection results in MOTChallenge or KITTI format.** Without the input video source, the user can set `inVdoTyp` as 0. Also, the frame size and frame rate are required at `ovrdFrmSz` and `ovrdFrmRt` respectively. 
2. When the ROI image is not selected, an ROI image of the entire frame will be automatically created. If the user wants to select a specified ROI, set `selRoiFlg` to 1. The user can perform 3 choices: 1) press `p` to proceed to the frame that s/he wants to select ROI in, 2) press `Esc` and load the existing ROI image, 3) press `s` to select ROI in the current frame. The process of selecting ROI is described as follows: 
   1. A new window called `ROI selector` pops out. This should be the current frame that the user proceeds to.
   2. Click on the image around the ROI. A blue circle stands for each click and a white line connects two adjacent clicks. Note that in order to make a valid ROI, the last click should be close to the first click, i.e., two blue circles should overlap.

<div align="center">
    <img src="/pic/pic0.png", width="660">
</div>

   3. After the selection of ROI is done, click `o`. A binary mask image that shows the mask for ROI is created. 

<div align="center">
    <img src="/pic/pic1.jpg", width="640">
</div>

   4. During ROI selection, if mis-clicking on wrong places, the user can press `r`. All the markers will be cleared, and s/he can start over.
3. The user can choose to output plotted tracking results (as colored bounding boxes for different IDs) by setting `outVdoTyp` to 1 or 2. As a reminder, the input video source is required for plotting tracking results. 

<div align="center">
    <img src="/pic/pic2.jpg", width="640">
</div>

4. To improve the continuity of tracklets, each object is tracked by prediction of the Kalman filter for several frames even when the detection is missing. The predicted instances will be removed if no following tracklet is found. Here are some ways to fine-tune the performance: 
   1. To reduce false positives (short-living objects), the user can increase `trkNtrTmSecThld` (unit: second). 
   2. To reduce false negatives (gaps in trajectories), the user can increase `trkHypTmSecThld` (unit: second). 
   3. To reduce identity switches, the user can increase `trkMtchScrThld` and/or `trkNtrScrThld`. Intuitively, two nodes/instances are matched when the matching score is higher than this threshold. When the frame rate is low, we use the inverse of the distance between their foot points as the indicator. Thus, the unit is the inverse of the number of pixels when the frame rate is smaller than `trkFrmRtThld` (unit: fps). Otherwise, the unit is based on IOU (intersection over union). 
   
5. Finally, the algorithm assumes that both the object IDs and frame IDs are 1-based, which means they always start counting from 1. To change it to 0-based, the user needs to modify `ST_OBJ_ID` and `ST_FRM_CNT` defined in `ObjTrk.h` before compiling.

## Disclaimer
For any question you can contact [Zheng (Thomas) Tang](https://github.com/zhengthomastang).

