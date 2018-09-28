#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//! prints a matrix of uchar
static void prtMatUchar(cv::Mat oMat)
{
	for (int y = 0; y < oMat.rows; y++)
	{
		for (int x = 0; x < oMat.cols; x++)
		{
			std::printf("%03d ", oMat.at<uchar>(y, x));
		}
		std::printf("\n");
	}
}

//! prints a matrix of double
static void prtMatDbl(cv::Mat oMat)
{
	for (int y = 0; y < oMat.rows; y++)
	{
		for (int x = 0; x < oMat.cols; x++)
		{
			std::printf("%.3f ", oMat.at<double>(y, x));
		}
		std::printf("\n");
	}
}

//! plots depth map
static void pltDepMap(cv::Mat oDepMap, int nObjNum)
{
	if (nObjNum > 0)
	{
		cv::Mat oImg = cv::Mat::zeros(oDepMap.size(), CV_8UC1);
		int nStp = 255 / nObjNum;

		for (int y = 0; y < oDepMap.rows; y++)
		{
			for (int x = 0; x < oDepMap.cols; x++)
			{
				oImg.at<uchar>(y, x) = oDepMap.at<int>(y, x) * nStp;
			}
		}

		//cv::imwrite(".\\data\\depmap.jpg", oImg);	// in Windows
		cv::imwrite("./data/depmap.jpg", oImg);	// in Linux
	}
}

//! compares the y coordinates of two points
static bool cmpPtY(cv::Point2f oPt1, cv::Point2f oPt2)	// used to sort pixel points
{
	return oPt1.y > oPt2.y;
}

//! calculates intersection over union (IOU) of two bounding boxes
static double calcBBoxIou(cv::Rect2f oBBox1, cv::Rect2f oBBox2)
{
	double fIntxnArea, fBBox1Area, fBBox2Area;

	// determine the (x, y)-coordinates of the intersection rectangle
	double fTopLftX = std::max(oBBox1.x, oBBox2.x);
	double fTopLftY = std::max(oBBox1.y, oBBox2.y);
	double fBtmRgtX = std::min((oBBox1.x + oBBox1.width), (oBBox2.x + oBBox2.width));
	double fBtmRgtY = std::min((oBBox1.y + oBBox1.height), (oBBox2.y + oBBox2.height));

	// compute the area of intersection rectangle
	if ((fBtmRgtX > fTopLftX) && (fBtmRgtY > fTopLftY))
		fIntxnArea = (fBtmRgtX - fTopLftX) * (fBtmRgtY - fTopLftY);
	// there is no intersection
	else
		return 0.0;

	// compute the area of both bounding boxes
	fBBox1Area = oBBox1.width * oBBox1.height;
	fBBox2Area = oBBox2.width * oBBox2.height;

	// compute the intersection over union by taking the intersection area and dividing it by the sum of bbox1 + bbox2 areas - the interesection area
	return (fIntxnArea / (fBBox1Area + fBBox2Area - fIntxnArea));
}

//! validates bounding box
static bool valBBox(cv::Rect2f& oBBox, cv::Size oFrmSz, cv::Mat oImgRoi)
{
	if (oBBox.x < 0.0f) { oBBox.x = 0.0f; }
	if (oBBox.y < 0.0f) { oBBox.y = 0.0f; }
	if ((oBBox.x + oBBox.width) >= oFrmSz.width) { oBBox.width = oFrmSz.width - oBBox.x - 1; }
	if ((oBBox.y + oBBox.height) >= oFrmSz.height) { oBBox.height = oFrmSz.height - oBBox.y - 1; }
	if ((oBBox.x >= 0.0f) && (oBBox.x < oFrmSz.width) && (oBBox.y >= 0.0f) && (oBBox.y < oFrmSz.height) && (oBBox.width > 0.0f) && (oBBox.height > 0.0f) &&
		(0 < oImgRoi.at<uchar>(cv::Point(oBBox.x, oBBox.y))) && (0 < oImgRoi.at<uchar>(cv::Point((oBBox.x + oBBox.width), oBBox.y))) &&
		(0 < oImgRoi.at<uchar>(cv::Point(oBBox.x, (oBBox.y + oBBox.height)))) && (0 < oImgRoi.at<uchar>(cv::Point((oBBox.x + oBBox.width), (oBBox.y + oBBox.height)))))
		return true;
	else
		return false;
}