#include <iostream>
#include <math.h>
#include "RoiSel.h"

CRoiSel oRoiSel;

void on_mouse2( int event, int x, int y, int flags, void* )  // mouse event
{
	if (oRoiSel.getPlgnCmpl() == true)
		return;

	if (oRoiSel.chkImgLd() == false)
	{
		std::cout << "Error: on_mouse2(): image is unloaded" << std::endl;
		return;
	}

	if (event == CV_EVENT_FLAG_LBUTTON)
		oRoiSel.addNd(x, y);

	return;
}

int compPtDist(const int nX1, const int nY1, const int nX2, const int nY2)
{
	return (int)sqrt((float)(nX1 - nX2)*(nX1 - nX2) + (nY1 - nY2)*(nY1 - nY2));
}

void CRoiSel::addNd(int nX, int nY)
{
	cv::Point oCurrNd;
	oCurrNd.x = nX;
	oCurrNd.y = nY;
	cv::Point oPreNd;

	if ( m_voNd.size() > 2 &&   //if the node is the last one (close to the first one)
		compPtDist(m_voNd.begin()->x, m_voNd.begin()->y, oCurrNd.x, oCurrNd.y) < ND_RAD)
	{
		oPreNd = m_voNd[m_voNd.size() - 1];
		cv::line(m_oImgExp, oPreNd, oCurrNd, cv::Scalar(255, 255, 255), 2, CV_AA);
		cv::imshow("ROI selector", m_oImgExp);
		m_bPlgnCmpl = true;
	}
	else
	{
		m_voNd.push_back(oCurrNd);
		// std::cout << "current node(" << oCurrNd.x << "," << oCurrNd.y << ")" << std::endl;  //[debug]

		if (m_voNd.size() > 1)  // draw the line if the node is not the first one
		{
			oPreNd = m_voNd[m_voNd.size() - 2];
			// std::cout << "previous node(" << oPreNd.x << "," << oPreNd.y << ")" << std::endl << endl;  //[debug]
			cv::line(m_oImgExp, oPreNd, oCurrNd, cv::Scalar(255, 255, 255), 2, CV_AA);
			cv::imshow("ROI selector", m_oImgExp);
		}

		cv::circle(m_oImgExp, oCurrNd, ND_RAD, cv::Scalar(255, 0, 0), 1, CV_AA);  // draw the circle
		cv::imshow("ROI selector", m_oImgExp);
	}
}

void CRoiSel::genRoi(char* acROIName)
{
	std::vector<std::vector<cv::Point> > vvCtr;

	for (int nPtIter = 0; nPtIter < m_voNd.size(); nPtIter++)
	{
		m_voNd[nPtIter].x -= 10;
		m_voNd[nPtIter].y -= 10;
	}

	vvCtr.push_back(m_voNd);
	cv::drawContours(m_oImgRoi, vvCtr, 0, cv::Scalar(255, 255, 255), CV_FILLED);
	cv::imwrite(acROIName, m_oImgRoi);
	std::vector<std::vector<cv::Point> >().swap(vvCtr);
}

bool CRoiSel::selRoi(const cv::Mat& oImgOrig, CCfg& oCfg)
{
	std::cout << "Hot keys: \n"
		 << "\tESC - finish selecting ROI (region of interest)\n"
		 << "\tr - re-select ROI\n"
		 << "\tENTER - produce ROI image\n";

	cv::namedWindow("ROI selector", CV_WINDOW_NORMAL);

	m_oImgCp = oImgOrig.clone();
	m_oImgExp = cv::Mat::zeros(cv::Size((oCfg.getFrmSz().width + 20), (oCfg.getFrmSz().height + 20)), CV_8UC3);
	m_oImgCp.copyTo(m_oImgExp(cv::Rect(10, 10, m_oImgCp.cols, m_oImgCp.rows)));
	m_oImgRoi = cv::Mat::zeros(cv::Size(oCfg.getFrmSz().width, oCfg.getFrmSz().height), CV_8UC1);

	cv::imshow("ROI selector", m_oImgExp);
	cv::setMouseCallback("ROI selector", on_mouse2);  // register for mouse event

	while (1)
	{
		int nKey = cv::waitKey(0);	// read keyboard event

		if (nKey == 27 )
			break;

		if (nKey == 'r')  // reset the nodes
		{
			std::vector<cv::Point>().swap(m_voNd);
			m_bPlgnCmpl = false;
			// std::cout << "size of m_voNd: " << m_voNd.size() << std::endl;  //[debug]
			m_oImgCp = oImgOrig.clone();
			m_oImgExp = cv::Mat::zeros(cv::Size((oCfg.getFrmSz().width + 20), (oCfg.getFrmSz().height + 20)), CV_8UC3);
			m_oImgCp.copyTo(m_oImgExp(cv::Rect(10, 10, m_oImgCp.cols, m_oImgCp.rows)));
			cv::imshow("ROI selector", m_oImgExp);
		}

		if (nKey == 'o' && m_bPlgnCmpl == true)  //produce the ROI image
		{
			genRoi(oCfg.getInRoiPth());

			cv::namedWindow("ROI image", CV_WINDOW_NORMAL);
			cv::imshow("ROI image", m_oImgRoi);
			m_oImgRoi = cv::Mat::zeros(oCfg.getFrmSz(), CV_8UC1);
			break;
		}
	}

	cv::destroyWindow("ROI selector");
	cv::destroyWindow("ROI image");

	return true;
}
