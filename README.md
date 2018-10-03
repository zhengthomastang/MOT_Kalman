# MOT_Kalman

This repository contains our C++ implementation of (intra-camera) online multiple object tracking based on Kalman filtering. It is useful for efficient tracklet generation in data association. 

## Introduction

This package is designed for generating 2D tracklets in each single camera. A 2D Kalman-filter-based tracking-by-detection method is adopted. The package also includes tools for manual region of interest (ROI) selection. 

## Coding Structure
1. `./obj/` folder: Libraries of .cpp files
2. `./src/` folder: Source code
3. `./data/` folder: Example input files
   1. `cfg.json`: Configuration parameters in JSON format
   2. `vdo.avi`: Input video source
   3. `det.txt`: Detection results in MOTChallenge format

## How to Build
1. Download and make the OpenCV library.
2. Compile using g++ in Linux environment. If you are new to g++ compilation with OpenCV, please refer to this [link](http://answers.opencv.org/question/25642/how-to-compile-basic-opencv-program-in-c-in-ubuntu/). In command window, you can `cd` to the directory of `./src/` and use the following command to compile our source code, where `../bin` is the executable file generated.  
```g++ -I/usr/local/include/ -L/usr/local/lib/ -g -o ../bin main.cpp Cfg.cpp RoiSel.cpp ObjDet.cpp ObjTrk.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lopencv_video -lm```

## How to Use
1. Set the corresponding input/output paths in the configuration file if necessary. 
2. When running the program, a window called `current frame` will pop up first.
3. When the ROI image is not selected, an ROI image of the entire frame will be automatically created. If the user wants to select a specified ROI, set `selRoiFlg` to 1. The user can perform 3 choices: 1) press `p` to proceed to the frame that s/he wants to select ROI in, 2) press `Esc` and load the existing ROI image, 3) press `s` to select ROI in the current frame. The process of selecting ROI is described as follows: 
   1. A new window called `ROI selector` pops out. This should be the current frame that the user proceeds to.
   2. Click on the image around the ROI. A blue circle stands for each click and a white line connects two adjacent clicks. Note that in order to make a valid ROI, the last click should be close to the first click, i.e., two blue circles should overlap.
   3. After the selection of ROI is done, click `o`. A binary mask image that shows the mask for ROI is created. 
   4. During ROI selection, if mis-clicking on wrong places, the user can press `r`. All the markers will be cleared, and s/he can start over.
4. The user can choose to plot tracking results (as colorful bounding boxes for different IDs) at the output window by setting `pltTrkResFlg`. The past trajectories of foot points can also be plotted when `pltTrajTmSec` is larger than zero. 
5. To improve the continuity of tracklets, each object is tracked by prediction of Kalman filter for several frames even when the detection is missing. Here are some ways to fine-tune the performance: 
   1. To reduce false positives (short-living objects), the user can increase `trkNtrTmSecThld`. The unit is second. 
   2. To reduce false negatives (gaps in trajectories), the user can increase `trkHypTmSecThld`. The unit is second. 
   3. To reduce identity switches, when the frame rate is smaller than 5 fps, the user can decrease `trkDistRatThld`. The unit is the inverse of the number of pixels. Otherwise the user has to increase the IOU thresholds defined in `ObjTrk.h`. 

## Disclaimer
For any question you can contact [Zheng (Thomas) Tang](https://github.com/zhengthomastang).

