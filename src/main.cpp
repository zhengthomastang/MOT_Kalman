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
	bool bProcFlg = true;
	cv::VideoCapture oVdoCap;
	cv::VideoWriter oVdoWrt;
	cv::Mat oImgFrm, oImgRoi;
	cv::Size oFrmSz;
	std::vector<CDetNd> voDetNd;
	CObjDet oObjDet;
	CObjTrk oObjTrk;
	CCfg oCfg;

	// read configuration file
	if (2 < argc)
	{
		std::printf("usage: %s <cfg_file_path>\n", argv[0]);
		return 0;
	}
	else if (2 == argc)
		oCfg.ldCfgFl(argv[1]);
	else
		oCfg.ldCfgFl('\0');

    oImgFrm = cv::Mat(oFrmSz, CV_8UC3, cv::Scalar(0, 0, 0));

    // set starting frame count
	nFrmCnt = oCfg.getProcStFrmCnt();

	// read video source
	// from image files
	if (1 == oCfg.getInVdoTyp())
	{
		char acInImgNm[128] = { 0 };
		std::sprintf(acInImgNm, "%06d.jpg", nFrmCnt);
		char acInImgPth[128] = { 0 };
		std::sprintf(acInImgPth, oCfg.getInFrmFlrPth());
		std::strcat(acInImgPth, acInImgNm);
		oImgFrm = cv::imread(acInImgPth, CV_LOAD_IMAGE_COLOR);
	}
	// from video file
	else if (2 == oCfg.getInVdoTyp())
		oVdoCap = cv::VideoCapture(oCfg.getInVdoPth());

	// handle error in reading video source
	if (((1 == oCfg.getInVdoTyp()) && (oImgFrm.empty())) || ((2 == oCfg.getInVdoTyp()) && (!oVdoCap.isOpened())))
	{
		std::cout << "Error: The video is not captured properly" << std::endl;
		return 0;
	}

	// set frame size
	if (1 == oCfg.getInVdoTyp())
	{
		oFrmSz.width = oImgFrm.cols;
		oFrmSz.height = oImgFrm.rows;
	}
	else if (2 == oCfg.getInVdoTyp())
	{
		oFrmSz.width = (int)oVdoCap.get(CV_CAP_PROP_FRAME_WIDTH);
		oFrmSz.height = (int)oVdoCap.get(CV_CAP_PROP_FRAME_HEIGHT);
	}
	else if (0 == oCfg.getInVdoTyp())
    {
        oFrmSz = oCfg.getOvrdFrmSz();
        oImgFrm = cv::Mat(oFrmSz, CV_8UC3, cv::Scalar(0, 0, 0));
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
	if (2 == oCfg.getInVdoTyp())
		oCfg.setFrmRt(oVdoCap.get(CV_CAP_PROP_FPS));
	else if ((1 == oCfg.getInVdoTyp()) || (0 == oCfg.getInVdoTyp()))
		oCfg.setFrmRt(oCfg.getOvrdFrmRt());

	// select ROI
	if (oCfg.getSelRoiFlg())
	{
		oRoiSel.selRoi(oImgFrm, oCfg.getInRoiPth());
		oImgRoi = cv::imread(oCfg.getInRoiPth(), CV_LOAD_IMAGE_GRAYSCALE);
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

	// regular tracking
	std::printf("\nStart regular tracking...\n");
	while (true)
	{
		// run till the end of the video
		if ((0 < oCfg.getProcFrmNum()) && (oCfg.getProcFrmNum() < nProcFrmNum))
			break;

		// showing frame count and time stamp
		std::printf("tracking - frame %06d\n", nFrmCnt);

		// initialize at the beginning
		if (oCfg.getProcStFrmCnt() == nFrmCnt)
		{
			// initialize object detector
			oObjDet.initialize(oCfg, oImgRoi);

			// initialize tracker
            oObjTrk.initialize(oCfg);
		}

		if (oCfg.getProcStFrmCnt() <= nFrmCnt)
		{
			// run object detection
			if (!oObjDet.process(voDetNd, nFrmCnt))
				bProcFlg = false;

			// run tracking
            oObjTrk.process(voDetNd, nFrmCnt);

			if (!bProcFlg)
				break;

			nProcFrmNum++;
		}

		nFrmCnt++;
	}

    // output tracking results
    oObjTrk.output();

	return 0;
}
