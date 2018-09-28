# SCT_Kalman

This repository contains our source code of C++ implementation of 2D single-camera tracking based on Kalman filtering.

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
2. Compile using g++ in Linux environment. 

## How to Use
1. Set the corresponding input/output paths in the configuration file if necessary. 
2. When running the program, a window called `current frame` will pop up first.
3. When the ROI image is not selected, an ROI image of the entire frame will be automatically created. If the user wants to select a specified ROI, set `selRoiFlg` to 1. The user can perform 3 choices: 1) press `p` to proceed to the frame that s/he wants to select ROI in, 2) press `Esc` and load the existing ROI image, 3) press `s` to select ROI in the current frame. The process of selecting ROI is described as follows: 
   1. A new window called `ROI selector` pops out. This should be the current frame that the user proceeds to.
   2. Click on the image around the ROI. A blue circle stands for each click and a white line connects two adjacent clicks. Note that in order to make a valid ROI, the last click should be close to the first click, i.e., two blue circles should overlap.
   3. After the selection of ROI is done, click `o`. A binary mask image that shows the mask for ROI is created. 
   4. During ROI selection, if mis-clicking on wrong places, the user can press `r`. All the markers will be cleared, and s/he can start over.
4. The user can choose to plot tracking results (as colorful bounding boxes for different IDs) at the output window by setting “pltTrkResFlg”. The past trajectories of foot points can also be plotted when `pltTrajTmSec` is larger than zero. To improve the continuity of tracklets, each object is tracked by prediction of Kalman filter for several frames even when the detection is missing. This may lead to some false positives. The balance is controlled by `trkHypTmSecThld`. 

## Disclaimer
For any question you can contact [Zheng (Thomas) Tang](https://github.com/zhengthomastang).

