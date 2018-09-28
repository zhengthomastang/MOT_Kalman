//-----------------------------------------------------------------------------------
//
//  Copyright (c) 2018 Zheng Tang <tangzhengthomas@gmail.com>.  All rights reserved.
//
//  Description:
//      Implementation of Kalman-filter-based (2D) single-camera tracking
//
//-----------------------------------------------------------------------------------

#include "Cfg.h"
#include "RoiSel.h"
#include "ObjDet.h"
#include "ObjTrk.h"

int main(int argc, char *argv[])
{
	int nFrmCnt, nProcFrmNum = 0;
	cv::VideoCapture oVdoCap;
	cv::VideoWriter oVdoWrt;
	cv::Mat oImgFrm, oImgRoi, oImgBg;
	cv::Size oFrmSz;
	std::vector<CDetNd> voDetNd;
	CObjDet oObjDet;
	CObjTrk oObjTrk;
	CCfg oCfg;
	oCfg.loadCfgFile();

	if (oCfg.getBs1StFrmCntFlg())
		nFrmCnt = 1;
	else
		nFrmCnt = 0;

	cv::namedWindow("current frame", CV_WINDOW_NORMAL);

	// read video source
	// from video file
	if (0 == oCfg.getInVdoTyp())
		oVdoCap = cv::VideoCapture(oCfg.getInVdoPth());
	// from image files
	else if (1 == oCfg.getInVdoTyp())
	{
		char acInImgNm[128] = { 0 };
		std::sprintf(acInImgNm, "%06d.jpg", nFrmCnt);
 		char acInImgPth[128] = { 0 };
 		std::sprintf(acInImgPth, oCfg.getInFrmFlrPth());
  		std::strcat(acInImgPth, acInImgNm);
		oImgFrm = cv::imread(acInImgPth, CV_LOAD_IMAGE_COLOR);
	}

	// handle error in reading video source
	if ((0 == oCfg.getInVdoTyp()) && (!oVdoCap.isOpened()))
	{
		std::cout << "Error: The video is not captured properly" << std::endl;
		return 0;
	}

	// set frame size
	if (0 == oCfg.getInVdoTyp())
	{
		oFrmSz.width = (int)oVdoCap.get(CV_CAP_PROP_FRAME_WIDTH);
		oFrmSz.height = (int)oVdoCap.get(CV_CAP_PROP_FRAME_HEIGHT);
	}
	else
	{
		oFrmSz.width = oImgFrm.cols;
		oFrmSz.height = oImgFrm.rows;
	}

	// resize frame if necessary
	if (0 >= oCfg.getRszFrmHei())
		oCfg.setFrmSz(oFrmSz);
	else
	{
		oFrmSz = cv::Size((((float)oFrmSz.width / (float)oFrmSz.height) * oCfg.getRszFrmHei()), oCfg.getRszFrmHei());
		oCfg.setFrmSz(oFrmSz);
	}

	// set video frame rate
	if (0 == oCfg.getInVdoTyp())
		oCfg.setFrmRt(oVdoCap.get(CV_CAP_PROP_FPS));
	else
		oCfg.setFrmRt(oCfg.getOvrdFrmRt());

    // set background image
    if (0 == oCfg.getInVdoTyp())
        oVdoCap >> oImgBg;
    else
        oImgBg = oImgFrm.clone();

    if (0 < oCfg.getRszFrmHei())
        cv::resize(oImgBg, oImgBg, oFrmSz);

	// select ROI
	if (oCfg.getSelRoiFlg())
	{
		while (true)
		{
			int nKeyboardIn = cv::waitKey(0);	// read keyboard event

			if (nKeyboardIn == 'p')	// proceed to next frame
			{
				if (0 == oCfg.getInVdoTyp())
					oVdoCap >> oImgFrm;
				else
				{
					char acInImgNm[128] = { 0 };
    					std::sprintf(acInImgNm, "%06d.jpg", nFrmCnt);
					char acInImgPth[128] = { 0 };
 					std::sprintf(acInImgPth, oCfg.getInFrmFlrPth());
					std::strcat(acInImgPth, acInImgNm);
  					oImgFrm = cv::imread(acInImgPth, CV_LOAD_IMAGE_COLOR);
					nFrmCnt++;
				}

				if (oImgFrm.empty())
				{
					std::cout << "Error: Reach the end of the video." << std::endl;
					return 0;
				}

				if (0 < oCfg.getRszFrmHei())
					cv::resize(oImgFrm, oImgFrm, oFrmSz);

				cv::imshow("current frame", oImgFrm);
			}

			if (nKeyboardIn == 27)	// press "Esc" to terminate
			{
				cv::destroyWindow("current frame");
				break;
			}

			if (nKeyboardIn == 's')			// select ROI
			{
				if (oImgFrm.empty())
				{
					std::cout << "Error: Please proceed to the frame to select ROI first." << std::endl;
					return 0;
				}

				oRoiSel.selRoi(oImgFrm, oCfg);
				cv::destroyWindow("current frame");
				break;
			}

			nFrmCnt++;
		}

		oImgRoi = cv::imread(oCfg.getInRoiPth(), CV_LOAD_IMAGE_GRAYSCALE);

		// reset video capture
		if (0 == oCfg.getInVdoTyp())
			oVdoCap = cv::VideoCapture(oCfg.getInVdoPth());
	}
	else
	{
		oImgRoi = cv::imread(oCfg.getInRoiPth(), CV_LOAD_IMAGE_GRAYSCALE);

		if (oImgRoi.empty())	// if ROI image does not exist, create a non-constraint ROI image
		{
			oImgRoi = cv::Mat(oFrmSz, CV_8UC1, cv::Scalar_<uchar>(255));
			cv::imwrite(oCfg.getInRoiPth(), oImgRoi);
		}
	}

	// set the ROI area
	oCfg.setRoiArea(cv::countNonZero(oImgRoi));

	// create video writer for output video
	if (oCfg.getOutVdoFlg())
	{
		if (0 == oCfg.getInVdoTyp())
			oVdoWrt = cv::VideoWriter(oCfg.getOutVdoPth(), CV_FOURCC('M', 'P', '4', '2'), (double)oVdoCap.get(CV_CAP_PROP_FPS), oFrmSz);
		else
			oVdoWrt = cv::VideoWriter(oCfg.getOutVdoPth(), CV_FOURCC('M', 'P', '4', '2'), oCfg.getFrmRt(), oFrmSz);
	}

	// create directory for output images
	if (oCfg.getOutImgFlg())
		//_mkdir(oCfg.getOutImgFlrPth());	// in Windows
		mkdir(oCfg.getOutImgFlrPth(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);	// in Linux

	// reset video capture
	if (0 == oCfg.getInVdoTyp())
		oVdoCap = cv::VideoCapture(oCfg.getInVdoPth());

	// set starting frame count
	if (oCfg.getBs1StFrmCntFlg())
		nFrmCnt = 1;
	else
		nFrmCnt = 0;

	// regular tracking
	cv::namedWindow("current frame", CV_WINDOW_NORMAL);
	std::printf("\nStart regular tracking...\n");
	while (true)
	{
		// read video frame
		if (0 == oCfg.getInVdoTyp())
			oVdoCap >> oImgFrm;
		else
		{
			char acInImgNm[128] = { 0 };
			std::sprintf(acInImgNm, "%06d.jpg", nFrmCnt);
			char acInImgPth[128] = { 0 };
			std::sprintf(acInImgPth, oCfg.getInFrmFlrPth());
			std::strcat(acInImgPth, acInImgNm);
			oImgFrm = cv::imread(acInImgPth, CV_LOAD_IMAGE_COLOR);
		}

		// resize video frame if necessary
		if (0 < oCfg.getRszFrmHei())
			cv::resize(oImgFrm, oImgFrm, oFrmSz);

		// run till the end of the video
		if ((oImgFrm.empty()) || ((0 < oCfg.getProcFrmNum()) && (oCfg.getProcFrmNum() < nProcFrmNum)))
			break;

		// manually end the process by pressing "Esc"
		int nKeyboardIn = cv::waitKey(1);	// read keyboard event
		if (nKeyboardIn == 27)
			break;

		// showing frame count and time stamp
		std::printf("frame %06d\n", nFrmCnt);

		// initialize at the beginning
		if (oCfg.getProcStFrmCnt() == nFrmCnt)
		{
			// initialize object detector
			oObjDet.initialize(oCfg, oImgRoi);

			// initialize tracker (not for self-calibration)
			if (oCfg.getProcTrkFlg())
				oObjTrk.initialize(oCfg);
		}

		if (oCfg.getProcStFrmCnt() <= nFrmCnt)
		{
			// run object detection
			oObjDet.process(voDetNd, nFrmCnt);

			// run tracking
			if (oCfg.getProcTrkFlg())
				oObjTrk.process(voDetNd, nFrmCnt);

			// output object detection results
				oObjDet.output(oImgFrm, voDetNd);

			// output tracking results
			if (oCfg.getProcTrkFlg())
				oObjTrk.output(oImgFrm);

			cv::imshow("current frame", oImgFrm);

			// write (plotted) frame to output video
			if (oCfg.getOutVdoFlg())
				oVdoWrt.write(oImgFrm);

			// save output (plotted) frame image
			if (oCfg.getOutImgFlg())
			{
				char acOutImgNm[128] = { 0 };
				std::sprintf(acOutImgNm, "%06d.jpg", nFrmCnt);
 				char acOutImgPth[128] = { 0 };
				std::sprintf(acOutImgPth, oCfg.getOutImgFlrPth());
				std::strcat(acOutImgPth, acOutImgNm);
				cv::imwrite(acOutImgPth, oImgFrm);
			}

			nProcFrmNum++;
		}

		nFrmCnt++;
	}

	cv::destroyWindow("current frame");

	return 0;
}
