#include "ObjTrk.h"

bool cmpDep2d(CCandNd oCandNd0, CCandNd oCandNd1)	// used to sort object nodes in a list
{
	// compare the y coordinates of the foot points
	return oCandNd0.get2dFtPt().y < oCandNd1.get2dFtPt().y;
}

bool cmpFrmCnt(CTrkNd oTrkNd0, CTrkNd oTrkNd1)	// used to sort object nodes in a list
{
	// compare the frame counts
	return oTrkNd0.getFrmCnt() < oTrkNd1.getFrmCnt();
}

CTraj::CTraj(void)
{
	std::vector<int>().swap(m_vnTrajFrmCnt);
	std::vector<cv::Rect2f>().swap(m_voTrajBBox);
	std::vector<cv::Point2f>().swap(m_voTraj2dFtPt);
	std::vector<bool>().swap(m_vbTrajPredFlg);
}

CTraj::CTraj(std::vector<int> vnFrmCnt, std::vector<cv::Rect2f> voBBox, std::vector<cv::Point2f> vo2dFtPt, std::vector<bool> vbPredFlg)
{
	setTrajFrmCnts(vnFrmCnt);
	setTrajBBoxs(voBBox);
	setTraj2dFtPts(vo2dFtPt);
	setTrajPredFlgs(vbPredFlg);
}

CTraj::~CTraj(void)
{
	std::vector<int>().swap(m_vnTrajFrmCnt);
	std::vector<cv::Rect2f>().swap(m_voTrajBBox);
	std::vector<cv::Point2f>().swap(m_voTraj2dFtPt);
	std::vector<bool>().swap(m_vbTrajPredFlg);
}

CCandNd::CCandNd(void)
{
	setDetNd(CDetNd());
	setSt8(ND_ST8_DFLT);
	setMtchTrkFlg(false);
	std::vector<cv::Rect2f>().swap(m_voMtchBBox);
}

CCandNd::CCandNd(CDetNd oDetNd, int nSt8)
{
	setDetNd(oDetNd);
	setBBox(oDetNd.getDetBBox());
	set2dFtPt(cv::Point2f((m_oBBox.x + (m_oBBox.width / 2.0f)), (m_oBBox.y + m_oBBox.height)));
	setSt8(nSt8);
	setMtchTrkFlg(false);
	std::vector<cv::Rect2f>().swap(m_voMtchBBox);
}

CCandNd::~CCandNd(void)
{
	std::vector<cv::Rect2f>().swap(m_voMtchBBox);
}

CTrkNd::CTrkNd(void)
{
	setDetNd(CDetNd());
	setId(-1);
	setNtrFrmCnt(-1);
	setMtchCandFlg(false);
	setPredFlg(false);
	setVis(1.0);
	setDepIdx(-1);
}

CTrkNd::CTrkNd(int nFrmCnt, int nId, cv::Rect2f oBBox)
{
    setFrmCnt(nFrmCnt);
    setId(nId);
    setBBox(oBBox);
}

CTrkNd::CTrkNd(CCandNd oCandNd, int nSt8, int nId, int nNtrFrmCnt, cv::KalmanFilter oKF)
{
	setCandNd(oCandNd, nSt8);
	setId(nId);
	setNtrFrmCnt(nNtrFrmCnt);
	setMtchCandFlg(false);
	setPredFlg(false);
	setVis(1.0);
	setDepIdx(-1);
	setKF(oKF);
}

CTrkNd::~CTrkNd(void)
{

}

CObjTrk::CObjTrk(void)
{
	// maximum object ID
	m_nMaxId = INT_MAX;

	// list of candidate nodes in current frame
	std::vector<CCandNd>().swap(m_voCurrCandNd);

	// list of candidate nodes in previous frame
	std::vector<CCandNd>().swap(m_voPrevCandNd);

	// list of tracking nodes in current frame
	std::vector<CTrkNd>().swap(m_voCurrTrkNd);

	// list of tracking nodes by prediction
	std::vector<CTrkNd>().swap(m_voPredNd);
}

CObjTrk::~CObjTrk(void)
{
	// list of candidate nodes in current frame
	std::vector<CCandNd>().swap(m_voCurrCandNd);;

	// list of candidate nodes in previous frame
	std::vector<CCandNd>().swap(m_voPrevCandNd);

	// list of tracking nodes in current frame
	std::vector<CTrkNd>().swap(m_voCurrTrkNd);

	// list of tracking nodes by prediction
	std::vector<CTrkNd>().swap(m_voPredNd);
}

void CObjTrk::initialize(CCfg oCfg)
{
	// configuration parameters
	m_oCfg = oCfg;

	// vector of colors of bounding boxes for plotting tracking results
	std::vector<cv::Scalar>().swap(m_voBBoxClr);
	m_voBBoxClr.push_back(cv::Scalar(255, 0, 0)); m_voBBoxClr.push_back(cv::Scalar(0, 255, 0)); m_voBBoxClr.push_back(cv::Scalar(0, 0, 255));
	m_voBBoxClr.push_back(cv::Scalar(255, 255, 0)); m_voBBoxClr.push_back(cv::Scalar(255, 0, 255)); m_voBBoxClr.push_back(cv::Scalar(0, 255, 255));
	m_voBBoxClr.push_back(cv::Scalar(63, 127, 255)); m_voBBoxClr.push_back(cv::Scalar(255, 63, 127)); m_voBBoxClr.push_back(cv::Scalar(127, 255, 63));
	m_voBBoxClr.push_back(cv::Scalar(63, 255, 127)); m_voBBoxClr.push_back(cv::Scalar(255, 127, 63)); m_voBBoxClr.push_back(cv::Scalar(127, 63, 255));
	m_voBBoxClr.push_back(cv::Scalar(255, 255, 127)); m_voBBoxClr.push_back(cv::Scalar(255, 127, 255)); m_voBBoxClr.push_back(cv::Scalar(127, 255, 255));
	m_voBBoxClr.push_back(cv::Scalar(0, 255, 127)); m_voBBoxClr.push_back(cv::Scalar(127, 0, 255)); m_voBBoxClr.push_back(cv::Scalar(255, 127, 0));
	m_voBBoxClr.push_back(cv::Scalar(0, 127, 255)); m_voBBoxClr.push_back(cv::Scalar(127, 255, 0)); m_voBBoxClr.push_back(cv::Scalar(255, 0, 127));
	m_voBBoxClr.push_back(cv::Scalar(0, 0, 127)); m_voBBoxClr.push_back(cv::Scalar(0, 127, 0)); m_voBBoxClr.push_back(cv::Scalar(127, 0, 0));
	m_voBBoxClr.push_back(cv::Scalar(63, 127, 0)); m_voBBoxClr.push_back(cv::Scalar(0, 63, 127)); m_voBBoxClr.push_back(cv::Scalar(127, 0, 63));
	m_voBBoxClr.push_back(cv::Scalar(63, 0, 127)); m_voBBoxClr.push_back(cv::Scalar(0, 127, 63)); m_voBBoxClr.push_back(cv::Scalar(127, 63, 0));
	m_voBBoxClr.push_back(cv::Scalar(127, 0, 127)); m_voBBoxClr.push_back(cv::Scalar(0, 127, 127)); m_voBBoxClr.push_back(cv::Scalar(127, 127, 0));
	m_voBBoxClr.push_back(cv::Scalar(255, 127, 191)); m_voBBoxClr.push_back(cv::Scalar(255, 191, 127)); m_voBBoxClr.push_back(cv::Scalar(191, 127, 255));
	m_voBBoxClr.push_back(cv::Scalar(191, 255, 127)); m_voBBoxClr.push_back(cv::Scalar(127, 255, 191)); m_voBBoxClr.push_back(cv::Scalar(127, 191, 255));
	m_voBBoxClr.push_back(cv::Scalar(63, 63, 255)); m_voBBoxClr.push_back(cv::Scalar(255, 63, 63)); m_voBBoxClr.push_back(cv::Scalar(63, 255, 63));
	m_voBBoxClr.push_back(cv::Scalar(95, 159, 31)); m_voBBoxClr.push_back(cv::Scalar(31, 159, 95)); m_voBBoxClr.push_back(cv::Scalar(159, 31, 95));
	m_voBBoxClr.push_back(cv::Scalar(95, 31, 159)); m_voBBoxClr.push_back(cv::Scalar(31, 95, 159));  m_voBBoxClr.push_back(cv::Scalar(159, 95, 31));
	m_voBBoxClr.push_back(cv::Scalar(223, 31, 223)); m_voBBoxClr.push_back(cv::Scalar(223, 223, 31)); m_voBBoxClr.push_back(cv::Scalar(31, 223, 223));
	m_voBBoxClr.push_back(cv::Scalar(31, 63, 223)); m_voBBoxClr.push_back(cv::Scalar(63, 31, 223)); m_voBBoxClr.push_back(cv::Scalar(223, 31, 63));
	m_voBBoxClr.push_back(cv::Scalar(31, 223, 63)); m_voBBoxClr.push_back(cv::Scalar(63, 223, 31)); m_voBBoxClr.push_back(cv::Scalar(223, 63, 31));
	m_voBBoxClr.push_back(cv::Scalar(63, 159, 63)); m_voBBoxClr.push_back(cv::Scalar(159, 63, 63)); m_voBBoxClr.push_back(cv::Scalar(63, 63, 159));
	m_voBBoxClr.push_back(cv::Scalar(31, 0, 127)); m_voBBoxClr.push_back(cv::Scalar(0, 127, 31)); m_voBBoxClr.push_back(cv::Scalar(127, 31, 0));
	m_voBBoxClr.push_back(cv::Scalar(31, 127, 0)); m_voBBoxClr.push_back(cv::Scalar(0, 31, 127)); m_voBBoxClr.push_back(cv::Scalar(127, 0, 31));
	m_voBBoxClr.push_back(cv::Scalar(63, 191, 191)); m_voBBoxClr.push_back(cv::Scalar(63, 191, 191)); m_voBBoxClr.push_back(cv::Scalar(191, 191, 63));
	m_voBBoxClr.push_back(cv::Scalar(0, 63, 191)); m_voBBoxClr.push_back(cv::Scalar(0, 191, 63)); m_voBBoxClr.push_back(cv::Scalar(191, 63, 0));
	m_voBBoxClr.push_back(cv::Scalar(191, 0, 63,0)); m_voBBoxClr.push_back(cv::Scalar(63, 0, 191)); m_voBBoxClr.push_back(cv::Scalar(63, 191, 0));

	// current frame count
	m_nFrmCnt = oCfg.getProcStFrmCnt();

	// maximum object ID
	m_nMaxId = ST_OBJ_ID;

	// ROI image
	m_oImgRoi = cv::imread(m_oCfg.getInRoiPth(), CV_LOAD_IMAGE_GRAYSCALE);

	// list of candidate nodes in current frame
	std::vector<CCandNd>().swap(m_voCurrCandNd);

	// list of candidate nodes in previous frame
	std::vector<CCandNd>().swap(m_voPrevCandNd);

	// list of tracking nodes in current frame
	std::vector<CTrkNd>().swap(m_voCurrTrkNd);

	// list of tracking nodes by 3D prediction
	std::vector<CTrkNd>().swap(m_voPredNd);

	// output text file of tracking results
	if (0 == m_oCfg.getOutTrkTyp())
    {
        FILE* pfOutTrkTxt = std::fopen(m_oCfg.getOutTrkTxtPth(), "w");
        std::fclose(pfOutTrkTxt);
    }
	// output folder of tracking results
	else if (1 == m_oCfg.getOutTrkTyp())
    {
		std::sprintf(m_acOutTrkFlrPth, oCfg.getOutTrkFlrPth());
        //_mkdir(m_acOutTrkFlrPth);	// in Windows
		mkdir(m_acOutTrkFlrPth, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);	// in Linux

		char acInDetTxtNm[128] = { 0 };
		char acInDetTxtPth[128] = { 0 };
		char acOutTrkTxtNm[128] = { 0 };
		char acOutTrkTxtPth[128] = { 0 };
		int nFrmCnt = ST_FRM_CNT;
		FILE* pfInDetTxt;
		FILE* pfOutTrkTxt;

		while (true)
		{
			std::sprintf(acInDetTxtNm, "%06d.txt", nFrmCnt);
			std::sprintf(acInDetTxtPth, m_oCfg.getInDetFlrPth());
			std::strcat(acInDetTxtPth, acInDetTxtNm);
			pfInDetTxt = std::fopen(acInDetTxtPth, "r");
			if (pfInDetTxt == NULL)
				break;
			std::fclose(pfInDetTxt);

			std::sprintf(acOutTrkTxtNm, "%06d.txt", nFrmCnt);
			std::sprintf(acOutTrkTxtPth, m_acOutTrkFlrPth);
			std::strcat(acOutTrkTxtPth, acOutTrkTxtNm);
			pfOutTrkTxt = std::fopen(acOutTrkTxtPth, "wb");
			std::fclose(pfOutTrkTxt);

			nFrmCnt++;
		}
    }
}

void CObjTrk::process(std::vector<CDetNd>& voDetNd, int nFrmCnt)
{
	// current frame count
	m_nFrmCnt = nFrmCnt;

	// sort objects by depth
	std::sort(m_voCurrTrkNd.begin(), m_voCurrTrkNd.end(), cmpDep2d);

	// create depth map of objects
	cv::Mat oDepMapTrk = genDepMap(m_voCurrTrkNd);

	for (std::vector<CTrkNd>::iterator it = m_voCurrTrkNd.begin(); it != m_voCurrTrkNd.end(); ++it)
	{
		// set the value of depth index according to the sorted list of tracking nodes
		it->setDepIdx(it - m_voCurrTrkNd.begin() + 1);

		// calculate visibility
		double fVis = calcVis(it->getBBox(), it->getDepIdx(),  oDepMapTrk);
		it->setVis(fVis);

		if (VIS_OCCL_OBJ_THLD > fVis)
            it->setSt8(ND_ST8_OCCL);

		// clear matching flag
		it->setMtchCandFlg(false);
	}

	// clear the list of candidate nodes
	std::vector<CCandNd>().swap(m_voCurrCandNd);

	// initialize the list of candidate nodes
	initCandNdLs(voDetNd);

	// track by Kalman filter
	for (int i = 0; i < m_voCurrTrkNd.size(); i++)
		trkKF(m_voCurrTrkNd[i]);

	// for unmatched objects, use prediction to continue tracking for few frames
	trkPred();

	// update Kalman filter
	for (int i = 0; i < m_voCurrTrkNd.size(); i++)
		updKF(m_voCurrTrkNd[i]);

	// set the list of predicted candidate nodes using Kalman filter prediction
	initPredNdLs();

	// detect left nodes
	detLftNd();

	// detect entering tracking nodes
	detNtrTrkNd();

	// insert each current tracking node to its trajectory
    for (std::vector<CTrkNd>::iterator it = m_voCurrTrkNd.begin(); it != m_voCurrTrkNd.end(); ++it)
		it->addTrajNd(m_nFrmCnt, it->getBBox(), it->get2dFtPt(), it->getPredFlg());

	// set the list of candidate nodes in previous frame
	m_voPrevCandNd = m_voCurrCandNd;
}

void CObjTrk::output(void)
{
	// output the rest of the tracking nodes
	for (int i = m_voCurrTrkNd.size() - 1; i >= 0; i--)
	{
		if (0 == m_oCfg.getOutTrkTyp())
			outTxtMot(m_voCurrTrkNd[i]);
		else
			outTxtKitti(m_voCurrTrkNd[i]);
	}

    // sort the tracking results in MOTChallenge format
    if (0 == m_oCfg.getOutTrkTyp())
        sortTxtMot();

    if ((1 == m_oCfg.getOutVdoTyp()) || (2 == m_oCfg.getOutVdoTyp()))
    {
		cv::VideoCapture oVdoCap;

		// input video source
		if (2 == m_oCfg.getInVdoTyp())
			oVdoCap = cv::VideoCapture(m_oCfg.getInVdoPth());

		cv::VideoWriter oVdoWrt;

		// create directory for output images
		if (1 == m_oCfg.getOutVdoTyp())
			//_mkdir(m_oCfg.getOutImgFlrPth());	// in Windows
			mkdir(m_oCfg.getOutImgFlrPth(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);	// in Linux
		// create video writer
		else if (2 == m_oCfg.getOutVdoTyp())
			oVdoWrt = cv::VideoWriter(m_oCfg.getOutVdoPth(), CV_FOURCC('M', 'P', '4', '2'), m_oCfg.getFrmRt(), m_oCfg.getFrmSz());

        // text file for tracking results
        if (0 == m_oCfg.getOutTrkTyp())
            m_ifsOutTrkTxt.open(m_oCfg.getOutTrkTxtPth());
        // folder of tracking results
        else if (1 == m_oCfg.getOutTrkTyp())
            std::sprintf(m_acOutTrkFlrPth, m_oCfg.getOutTrkFlrPth());

        std::vector<CTrkNd> voTrkNd;
        int nFrmCnt = ST_FRM_CNT;
        cv::Mat oImgFrm;

        cv::namedWindow("current frame", CV_WINDOW_NORMAL);
        std::printf("\nOutput tracking results...\n");

        while (true)
        {
            std::printf("output - frame %06d\n", nFrmCnt);

            std::vector<CTrkNd>().swap(voTrkNd);
            bool bProcFlg;

			// read frame image
			if (1 == m_oCfg.getInVdoTyp())
			{
				char acInImgNm[128] = { 0 };
				char acInImgPth[128] = { 0 };
				std::sprintf(acInImgNm, "%06d.jpg", nFrmCnt);
				std::sprintf(acInImgPth, m_oCfg.getInFrmFlrPth());
				std::strcat(acInImgPth, acInImgNm);
				oImgFrm = cv::imread(acInImgPth, CV_LOAD_IMAGE_COLOR);
			}
			else if (2 == m_oCfg.getInVdoTyp())
				oVdoCap >> oImgFrm;

            // read object tracking results in MOTChallenge format
            if (0 == m_oCfg.getOutTrkTyp())
                bProcFlg = rdObjTrkMot(voTrkNd, nFrmCnt);
            // read object tracking  results in KITTI format
            else if (1 == m_oCfg.getOutTrkTyp())
                bProcFlg = rdObjTrkKitti(voTrkNd, nFrmCnt);

            // plot tracking bounding box(es) on the frame image
            pltTrkBBox(oImgFrm, nFrmCnt, voTrkNd);
            cv::imshow("current frame", oImgFrm);
			cv::waitKey(1);

			// output plotted image
			if (1 == m_oCfg.getOutVdoTyp())
			{
				char acOutImgNm[128] = { 0 };
				char acOutImgPth[128] = { 0 };
				std::sprintf(acOutImgNm, "%06d.jpg", nFrmCnt);
				std::sprintf(acOutImgPth, m_oCfg.getOutImgFlrPth());
				std::strcat(acOutImgPth, acOutImgNm);
				cv::imwrite(acOutImgPth, oImgFrm);
			}
			else if (2 == m_oCfg.getOutVdoTyp())
				oVdoWrt.write(oImgFrm);

            if (!bProcFlg)
                break;

            nFrmCnt++;
        }

        if (0 == m_oCfg.getOutTrkTyp())
            m_ifsOutTrkTxt.close();

        cv::destroyWindow("current frame");
    }
}

void CObjTrk::detNtrTrkNd(void)
{
	// entering node: not matched with any tracking node
	cv::Size2f oBBoxSzChg;
	for (int i = 0; i < m_voCurrCandNd.size(); i++)
	{
        if ((!m_voCurrCandNd[i].getMtchTrkFlg()) && (m_oCfg.getTrkNtrFrmNumThld() < m_voCurrCandNd[i].getMtchBBoxNum()))
        {
            cv::Rect2f oNtrBBox = m_voCurrCandNd[i].getBBox();

            if (valBBox(oNtrBBox, m_oCfg.getFrmSz(), m_oImgRoi))
            {
                // initialize new tracking node
                CTrkNd oTrkNd(m_voCurrCandNd[i], m_voCurrCandNd[i].getSt8(), m_nMaxId++, m_nFrmCnt, initKF(m_voCurrCandNd[i]));
				int nMtchBBoxNum = m_voCurrCandNd[i].getMtchBBoxNum();
				for (int j = 0; j < nMtchBBoxNum; j++)
				{
					cv::Rect2f oMtchBBox = m_voCurrCandNd[i].getMtchBBox(j);
					cv::Point2f oMtch2dFtPt = cv::Point2f((oMtchBBox.x + (oMtchBBox.width / 2.0f)), (oMtchBBox.y + oMtchBBox.height));
					oTrkNd.addTrajNd((m_nFrmCnt - nMtchBBoxNum + j), oMtchBBox, oMtch2dFtPt, false);
				}

                m_voCurrTrkNd.push_back(oTrkNd);
            }
        }
	}
}

void CObjTrk::detLftNd(void)
{
	std::vector<int> viLftTrkNd;
	std::vector<int>::iterator it;

	for (int i = m_voCurrTrkNd.size() - 1; i >= 0; i--)
	{
		// a pair of objects fully overlap with each other: erase the one with larger movement
		for (int j = i - 1; j >= 0; j--)
		{
			if (FULL_OVLP_IOU_THLD < calcBBoxIou(m_voCurrTrkNd[i].getBBox(), m_voCurrTrkNd[j].getBBox()))
			{
				if ((0 < m_voCurrTrkNd[i].getTrajLen()) && (0 < m_voCurrTrkNd[j].getTrajLen()))
				{
					int iLftTrkNd = (cv::norm(m_voCurrTrkNd[i].get2dFtPt() - m_voCurrTrkNd[i].getTraj2dFtPt(m_voCurrTrkNd[i].getTrajLen() - 1)) >
						cv::norm(m_voCurrTrkNd[j].get2dFtPt() - m_voCurrTrkNd[j].getTraj2dFtPt(m_voCurrTrkNd[j].getTrajLen() - 1))) ? i : j;
					it = find(viLftTrkNd.begin(), viLftTrkNd.end(), iLftTrkNd);
					if (viLftTrkNd.end() == it)
						viLftTrkNd.push_back(iLftTrkNd);
				}
				else if (0 >= m_voCurrTrkNd[j].getTrajLen())
				{
					it = find(viLftTrkNd.begin(), viLftTrkNd.end(), j);
					if (viLftTrkNd.end() == it)
						viLftTrkNd.push_back(j);
				}
				else if (0 >= m_voCurrTrkNd[i].getTrajLen())
				{
					it = find(viLftTrkNd.begin(), viLftTrkNd.end(), i);
					if (viLftTrkNd.end() == it)
						viLftTrkNd.push_back(i);
				}
			}
		}

		it = find(viLftTrkNd.begin(), viLftTrkNd.end(), i);

        int nPredFrmNum = m_voCurrTrkNd[i].getTrajPredNum();

		// the number of frames that the object is tracked as by prediction exceeds the threshold
		if ((nPredFrmNum > m_oCfg.getTrkPredFrmNumThld()) ||
			// the object is being occluded by other objects
			(ND_ST8_OCCL == m_voCurrTrkNd[i].getSt8()) ||
			// the current tracking node needs to be removed
			(viLftTrkNd.end() != it))
		{
            if (0 == m_oCfg.getOutTrkTyp())
                outTxtMot(m_voCurrTrkNd[i]);
            else
                outTxtKitti(m_voCurrTrkNd[i]);

			m_voCurrTrkNd.erase(m_voCurrTrkNd.begin() + i);

			if (viLftTrkNd.end() != it)
				viLftTrkNd.erase(it);
		}
	}
}

void CObjTrk::initPredNdLs(void)
{
	std::vector<CTrkNd>().swap(m_voPredNd);
	for (int i = 0; i < m_voCurrTrkNd.size(); i++)
	{
		// Kalman filter prediction
		CCandNd oPredCandNd;
		cv::Point2f oPred2dFtPt;
		cv::Size2f oPred2dBBoxSz;

		cv::Mat oKFSt8Vec = cv::Mat(6, 1, CV_64F);
		cv::Mat oKFMeasVec = cv::Mat(4, 1, CV_64F);

		m_voCurrTrkNd[i].setKFTmDiff(1.0 / m_oCfg.getFrmRt());
		oKFSt8Vec = m_voCurrTrkNd[i].predKF();

		oPred2dFtPt = cv::Point2f(oKFSt8Vec.at<double>(0, 0), oKFSt8Vec.at<double>(1, 0));
		oPred2dBBoxSz = cv::Size2f(oKFSt8Vec.at<double>(4, 0), oKFSt8Vec.at<double>(5, 0));
		if (!shfCandNd(m_voCurrTrkNd[i], oPredCandNd, oPred2dFtPt, oPred2dBBoxSz))
			oPredCandNd = m_voCurrTrkNd[i];

		// create new list of predicted node
		CTrkNd oPredTrkNd = m_voCurrTrkNd[i];
		oPredTrkNd.setCandNd(oPredCandNd, oPredTrkNd.getSt8());
		m_voPredNd.push_back(oPredTrkNd);
	}
}

cv::KalmanFilter CObjTrk::initKF(CCandNd oCandNd)
{
	cv::KalmanFilter oKF;

	// state space
	// state vector: 2dFtPt.x, 2dFtPt.y, 2dVel.x, 2dVel.y, 2dBBox.width, 2dBBox.height
	// measurement vector: 2dFtPt.x, 2dFtPt.y, 2dBBox.width, 2dBBox.height
	oKF.init(6, 4, 0, CV_64F);

	// Transition State Matrix A
	// dT: inverse of frame rate
	// [ 1 0 dT 0  0 0 ]
	// [ 0 1 0  dT 0 0 ]
	// [ 0 0 1  0  0 0 ]
	// [ 0 0 0  1  0 0 ]
	// [ 0 0 0  0  1 0 ]
	// [ 0 0 0  0  0 1 ]
	cv::setIdentity(oKF.transitionMatrix);

	// Measurement Matrix H
	// [ 1 0 0 0 0 0 ]
	// [ 0 1 0 0 0 0 ]
	// [ 0 0 0 0 1 0 ]
	// [ 0 0 0 0 0 1 ]
	oKF.measurementMatrix = cv::Mat::zeros(4, 6, CV_64F);
	oKF.measurementMatrix.at<double>(0, 0) = 1.0f;
	oKF.measurementMatrix.at<double>(1, 1) = 1.0f;
	oKF.measurementMatrix.at<double>(2, 4) = 1.0f;
	oKF.measurementMatrix.at<double>(3, 5) = 1.0f;

	// Process Noise Covariance Matrix Q
	// [ Ex 0  0    0 0    0  ]
	// [ 0  Ey 0    0 0    0  ]
	// [ 0  0  Ev_x 0 0    0  ]
	// [ 0  0  0    1 Ev_y 0  ]
	// [ 0  0  0    0 1    Ew ]
	// [ 0  0  0    0 0    Eh ]
	//cv::setIdentity(oKF.processNoiseCov, cv::Scalar(KF_PROC_NOISE_COV_FTPT));
	oKF.processNoiseCov.at<double>(0, 0) = KF_PROC_NOISE_COV_FTPT;
	oKF.processNoiseCov.at<double>(1, 1) = KF_PROC_NOISE_COV_FTPT;
	oKF.processNoiseCov.at<double>(2, 2) = KF_PROC_NOISE_COV_VEL;
	oKF.processNoiseCov.at<double>(3, 4) = KF_PROC_NOISE_COV_VEL;
	oKF.processNoiseCov.at<double>(4, 5) = KF_PROC_NOISE_COV_BBOX_SZ;
	oKF.processNoiseCov.at<double>(5, 5) = KF_PROC_NOISE_COV_BBOX_SZ;

	// Measurement Noise Covariance Matrix R
	cv::setIdentity(oKF.measurementNoiseCov, cv::Scalar(KF_MEAS_NOISE_COV));

	// Priori Error Estimate Covariance Matrix P'(k)
	cv::setIdentity(oKF.errorCovPre, cv::Scalar::all(KF_ERR_COV_PRE));

	cv::Mat oKFSt8Vec = cv::Mat(6, 1, CV_64F);
	oKFSt8Vec.at<double>(0, 0) = oCandNd.get2dFtPt().x;
	oKFSt8Vec.at<double>(1, 0) = oCandNd.get2dFtPt().y;
	oKFSt8Vec.at<double>(2, 0) = 0.0f;
	oKFSt8Vec.at<double>(3, 0) = 0.0f;
	oKFSt8Vec.at<double>(4, 0) = oCandNd.getBBox().width;
	oKFSt8Vec.at<double>(5, 0) = oCandNd.getBBox().height;

	// Corrected State x(k)
	oKF.statePost = oKFSt8Vec;

	return oKF;
}

void CObjTrk::trkKF(CTrkNd& oTrkNd)
{
	// the predicted nodes are created by Kalman filter prediction
	int iPred = -1;
	for (int i = 0; i < m_voPredNd.size(); i++)
	{
		if ((m_voPredNd[i].getId() == oTrkNd.getId()) && (!oTrkNd.getMtchCandFlg()))
		{
			iPred = i;
			break;
		}
	}

	// match the tracking node with a candidate node
	if (0 <= iPred)
	{
		cv::Rect2f oPredBBox = m_voPredNd[iPred].getBBox();
		if (valBBox(oPredBBox, m_oCfg.getFrmSz(), m_oImgRoi))
			mtchTrkCand(oTrkNd, oPredBBox, m_voPredNd[iPred].get2dFtPt());
	}
}

void CObjTrk::updKF(CTrkNd& oTrkNd)
{
	cv::Mat oKFMeasVec = cv::Mat(4, 1, CV_64F);

	oKFMeasVec.at<double>(0, 0) = oTrkNd.get2dFtPt().x;
	oKFMeasVec.at<double>(1, 0) = oTrkNd.get2dFtPt().y;
	oKFMeasVec.at<double>(2, 0) = oTrkNd.getBBox().width;
	oKFMeasVec.at<double>(3, 0) = oTrkNd.getBBox().height;

    oTrkNd.corrKF(oKFMeasVec);
}

void CObjTrk::trkPred(void)
{
	for (std::vector<CTrkNd>::iterator it = m_voCurrTrkNd.begin(); it != m_voCurrTrkNd.end(); ++it)
	{
		if (!it->getMtchCandFlg())
		{
            double fMtchScr, fMaxMtchScr = 0.0;
            int iMaxMtchScr = -1;

			cv::Size oFrmSz = m_oCfg.getFrmSz();

            for (int i = 0; i < m_voCurrCandNd.size(); i++)
            {
                if (!m_voCurrCandNd[i].getMtchTrkFlg())
                {
					if (m_oCfg.getTrkFrmRtThld() <= m_oCfg.getFrmRt())
						fMtchScr = calcBBoxIou(it->getBBox(), m_voCurrCandNd[i].getBBox());
					else
						fMtchScr = 1.0 / cv::norm(it->get2dFtPt() - m_voCurrCandNd[i].get2dFtPt());

                    if (fMtchScr > fMaxMtchScr)
                    {
                        fMaxMtchScr = fMtchScr;
                        iMaxMtchScr = i;
                    }
                }
            }

            // matched with an unmatched candidate node
            if (m_oCfg.getTrkMtchScrThld() < fMaxMtchScr)
            {
				it->setCandNd(m_voCurrCandNd[iMaxMtchScr], m_voCurrCandNd[iMaxMtchScr].getSt8());
				it->setMtchCandFlg(true);
				it->setPredFlg(false);
                it->resetTrajPredFlg();
				m_voCurrCandNd[iMaxMtchScr].setMtchTrkFlg(true);
			}
            // track by prediction
            else
            {
                // the predicted nodes are created by Kalman filter prediction
                int iPred = -1;
                for (int i = 0; i < m_voPredNd.size(); i++)
                {
                    if (m_voPredNd[i].getId() == it->getId())
                    {
                        iPred = i;
                        break;
                    }
                }

                if (0 <= iPred)
                {
                    if (ND_ST8_EDG == it->getSt8())
                        it->setCandNd(m_voPredNd[iPred], ND_ST8_EDG);
                    else
                        it->setCandNd(m_voPredNd[iPred], ND_ST8_DFLT);
                }

				it->setPredFlg(true);
            }
		}
		else
            it->resetTrajPredFlg();
	}
}

void CObjTrk::mtchTrkCand(CTrkNd& oTrkNd, cv::Rect2f oPredBBox, cv::Point2f oPred2dFtPt)
{
	if (valBBox(oPredBBox, m_oCfg.getFrmSz(), m_oImgRoi))
	{
		int iMaxMtchScr = -1;
		double fMtchScr, fMaxMtchScr = -1.0;

		for (int i = 0; i < m_voCurrCandNd.size(); i++)
		{
            if (!m_voCurrCandNd[i].getMtchTrkFlg())
            {
                // real-time processing: use IOU between bounding boxes as matching score
                // otherwise: use 2D distance between foot points as matching score
				cv::Size oFrmSz = m_oCfg.getFrmSz();

                if (m_oCfg.getTrkFrmRtThld()  <= m_oCfg.getFrmRt())
                    fMtchScr = calcBBoxIou(oPredBBox, m_voCurrCandNd[i].getBBox());
                else
					fMtchScr = 1.0 / cv::norm(oPred2dFtPt - m_voCurrCandNd[i].get2dFtPt());

                if (fMtchScr > fMaxMtchScr)
                {
                    fMaxMtchScr = fMtchScr;
                    iMaxMtchScr = i;
                }
			}
		}

		// matched with a candidate node
		if (m_oCfg.getTrkMtchScrThld() < fMaxMtchScr)
		{
			oTrkNd.setCandNd(m_voCurrCandNd[iMaxMtchScr], m_voCurrCandNd[iMaxMtchScr].getSt8());
			oTrkNd.setMtchCandFlg(true);
			oTrkNd.setPredFlg(false);
			m_voCurrCandNd[iMaxMtchScr].setMtchTrkFlg(true);
		}
	}
}

void CObjTrk::initCandNdLs(std::vector<CDetNd>& voDetNd)
{
	// all detected bounding box(es) are trusted
	for (int i = 0; i < voDetNd.size(); i++)
		m_voCurrCandNd.push_back(CCandNd(voDetNd[i], ND_ST8_NORM));

	// determine occluded-by-object state
	if (1 < m_voCurrCandNd.size())
		std::sort(m_voCurrCandNd.begin(), m_voCurrCandNd.end(), cmpDep2d);

	cv::Mat oCandDepMap = genDepMap(m_voCurrCandNd);

	for (std::vector<CCandNd>::iterator it = m_voCurrCandNd.begin(); it != m_voCurrCandNd.end(); ++it)
	{
		int iDep = it - m_voCurrCandNd.begin() + 1;
		if (VIS_OCCL_OBJ_THLD > calcVis(it->getBBox(), iDep, oCandDepMap))
			it->setSt8(ND_ST8_OCCL);
	}

	// determine occluded-by-edge state
	for (std::vector<CCandNd>::iterator it = m_voCurrCandNd.begin(); it != m_voCurrCandNd.end(); ++it)
	{
		// at the frame edge
		if ((FRM_EDG_BDTH > it->getBBox().x) || (FRM_EDG_BDTH > it->getBBox().y) ||
			((m_oCfg.getFrmSz().width - FRM_EDG_BDTH - 1) <= (it->getBBox().x + it->getBBox().width)) ||
			((m_oCfg.getFrmSz().height - FRM_EDG_BDTH - 1) <= (it->getBBox().y + it->getBBox().height)))
			it->setSt8(ND_ST8_EDG);
		else
		{
			cv::Mat oBBoxMsk = cv::Mat::zeros(m_oCfg.getFrmSz(), CV_8UC1);
			cv::rectangle(oBBoxMsk, it->getBBox(), cv::Scalar(255), -1);
			cv::dilate(oBBoxMsk, oBBoxMsk, cv::Mat(), cv::Point(-1, -1), FRM_EDG_BDTH);
			cv::bitwise_or(oBBoxMsk, m_oImgRoi, oBBoxMsk);
			// at the ROI edge
			if (cv::countNonZero(oBBoxMsk) > m_oCfg.getRoiArea())
				it->setSt8(ND_ST8_EDG);
		}
	}

	// set the number of matched frames by comparing with previous candidate nodes
	int iMaxNtrScr;
	double fNtrScr, fMaxNtrScr;

	for (int i = 0; i < m_voCurrCandNd.size(); i++)
	{
        fMaxNtrScr = 0.0;
        iMaxNtrScr = -1;
        for (int j = 0; j < m_voPrevCandNd.size(); j++)
        {
            if (!m_voPrevCandNd[j].getMtchTrkFlg())
            {
                if (m_oCfg.getTrkNtrFrmNumThld() >= m_voPrevCandNd[j].getMtchBBoxNum())
                {
					if (m_oCfg.getTrkFrmRtThld() <= m_oCfg.getFrmRt())
						fNtrScr = calcBBoxIou(m_voPrevCandNd[j].getBBox(), m_voCurrCandNd[i].getBBox());
					else
						fNtrScr = 1.0 / cv::norm(m_voPrevCandNd[j].get2dFtPt() - m_voCurrCandNd[i].get2dFtPt());

                    if ((m_oCfg.getTrkNtrScrThld() < fNtrScr) && (fMaxNtrScr < fNtrScr))
                    {
                        fMaxNtrScr = fNtrScr;
                        iMaxNtrScr = j;
                    }
                }
            }
        }

        // if matched with a previous candidate node, add to the trajectory
        if ((0 <= iMaxNtrScr) && (0.0 < fMaxNtrScr))
        {
            m_voCurrCandNd[i].setMtchBBoxs(m_voPrevCandNd[iMaxNtrScr].getMtchBBoxs());
            m_voCurrCandNd[i].addMtchBBox(m_voPrevCandNd[iMaxNtrScr].getBBox());
        }
    }
}

bool CObjTrk::shfCandNd(CCandNd oCandNd, CCandNd& oPredCandNd, cv::Point2f oPred2dFtPt, cv::Size2f oPred2dBBoxSz)
{
	float f2dPtXRat = (oCandNd.get2dFtPt().x - oCandNd.getBBox().x) / oCandNd.getBBox().width;
	float f2dPtYRat = (oCandNd.get2dFtPt().y - oCandNd.getBBox().y) / oCandNd.getBBox().height;

	cv::Rect2f oPredBBox((oPred2dFtPt.x - (f2dPtXRat * oPred2dBBoxSz.width)), (oPred2dFtPt.y - (f2dPtYRat * oPred2dBBoxSz.height)),
		oPred2dBBoxSz.width, oPred2dBBoxSz.height);

	if (valBBox(oPredBBox, m_oCfg.getFrmSz(), m_oImgRoi))
	{
		oPredCandNd.setDetNd(oCandNd);
		oPredCandNd.setBBox(oPredBBox);
		oPredCandNd.set2dFtPt(oPred2dFtPt);
		oPredCandNd.setSt8(oCandNd.getSt8());
		oPredCandNd.setMtchTrkFlg(oCandNd.getMtchTrkFlg());
		return true;
	}
	else
		return false;
}

cv::Mat CObjTrk::genDepMap(std::vector<CTrkNd> voTrkNd)
{
	cv::Mat oDepMap;
	oDepMap = cv::Mat::zeros(m_oCfg.getFrmSz(), cv::DataType<int>::type);
	int iDepBtm = INT_MAX;

	for (std::vector<CTrkNd>::iterator it = voTrkNd.begin(); it != voTrkNd.end(); ++it)
	{
		int iDep = it - voTrkNd.begin() + 1;
		cv::Rect2f oBBox = it->getBBox();
		if (valBBox(oBBox, m_oCfg.getFrmSz(), m_oImgRoi))
		{
			if ((m_oCfg.getFrmSz().height - FRM_EDG_BDTH - 1) <= (oBBox.y + oBBox.height))
				iDepBtm = iDep;

			for (int x = it->getBBox().x; x < (it->getBBox().x + it->getBBox().width); x++)
			{
				for (int y = it->getBBox().y; y < (it->getBBox().y + it->getBBox().height); y++)
				{
					if (iDep > oDepMap.at<int>(cv::Point(x, y)))
					{
						if (iDep > iDepBtm)
							oDepMap.at<int>(cv::Point(x, y)) = iDepBtm;
						else
							oDepMap.at<int>(cv::Point(x, y)) = iDep;
					}
				}
			}
		}
	}

	return oDepMap;
}

cv::Mat CObjTrk::genDepMap(std::vector<CCandNd> voCandNd)
{
	cv::Mat oDepMap;
	oDepMap = cv::Mat::zeros(m_oCfg.getFrmSz(), cv::DataType<int>::type);
	int iDepBtm = INT_MAX;

	for (std::vector<CCandNd>::iterator it = voCandNd.begin(); it != voCandNd.end(); ++it)
	{
		int iDep = it - voCandNd.begin() + 1;
		cv::Rect2f oBBox = it->getBBox();
		if (valBBox(oBBox, m_oCfg.getFrmSz(), m_oImgRoi))
		{
			if ((m_oCfg.getFrmSz().height - FRM_EDG_BDTH - 1) <= (oBBox.y + oBBox.height))
				iDepBtm = iDep;

			for (int x = it->getBBox().x; x < (it->getBBox().x + it->getBBox().width); x++)
			{
				for (int y = it->getBBox().y; y < (it->getBBox().y + it->getBBox().height); y++)
				{
					if (iDep > oDepMap.at<int>(cv::Point(x, y)))
					{
						if (iDep > iDepBtm)
							oDepMap.at<int>(cv::Point(x, y)) = iDepBtm;
						else
							oDepMap.at<int>(cv::Point(x, y)) = iDep;
					}
				}
			}
		}
	}

	return oDepMap;
}

double CObjTrk::calcVis(cv::Rect2f oBBox, int iDep, cv::Mat oDepMap)
{
	int nVisArea = 0;

	// the rectangle is in floating point format
	for (int x = oBBox.x + 1; x < (oBBox.x + oBBox.width - 2); x++)
	{
		for (int y = oBBox.y + 1; y < (oBBox.y + oBBox.height - 2); y++)
		{
			if (iDep >= oDepMap.at<int>(cv::Point(x, y)))
			{
				nVisArea++;
			}
		}
	}

	double fVis = (double)nVisArea / (double)(oBBox.width * oBBox.height);

	return ((fVis <= 1.0) ? fVis : 1.0);
}

void CObjTrk::outTxtMot(CTrkNd oTrkNd)
{
	FILE* pfOutTrkTxt = std::fopen(m_oCfg.getOutTrkTxtPth(), "a");

	for (int i = 0; i < oTrkNd.getTrajLen(); i++)
	{
	    if (!oTrkNd.getTrajPredFlg(i))
            std::fprintf(pfOutTrkTxt, "%d,%d,%.3f,%.3f,%.3f,%.3f,-1,-1,-1,-1\n",
                oTrkNd.getTrajFrmCnt(i), oTrkNd.getId(), oTrkNd.getTrajBBox(i).x, oTrkNd.getTrajBBox(i).y, oTrkNd.getTrajBBox(i).width, oTrkNd.getTrajBBox(i).height);
	}

    if (!oTrkNd.getPredFlg())
        std::fprintf(pfOutTrkTxt, "%d,%d,%.3f,%.3f,%.3f,%.3f,-1,-1,-1,-1\n",
            m_nFrmCnt, oTrkNd.getId(), oTrkNd.getBBox().x, oTrkNd.getBBox().y, oTrkNd.getBBox().width, oTrkNd.getBBox().height);

	std::fclose(pfOutTrkTxt);
}

void CObjTrk::outTxtKitti(CTrkNd oTrkNd)
{
    char acOutTrkTxtNm[128] = { 0 };
	char acOutTrkTxtPth[128] = { 0 };
	FILE* pfOutTrkTxt;

	for (int i = 0; i < oTrkNd.getTrajLen(); i++)
	{
	    if (!oTrkNd.getTrajPredFlg(i))
        {
            std::sprintf(acOutTrkTxtNm, "%06d.txt", oTrkNd.getTrajFrmCnt(i));
            std::sprintf(acOutTrkTxtPth, m_acOutTrkFlrPth);
            std::strcat(acOutTrkTxtPth, acOutTrkTxtNm);

            pfOutTrkTxt = std::fopen(acOutTrkTxtPth, "a");

            std::fprintf(pfOutTrkTxt, "%s 0.0 %d 0.0 %f %f %f %f 0.0 0.0 0.0 0.0 0.0 0.0 0.0\n", oTrkNd.getDetCls(), oTrkNd.getId(),
                   oTrkNd.getTrajBBox(i).x, oTrkNd.getTrajBBox(i).y, (oTrkNd.getTrajBBox(i).x + oTrkNd.getTrajBBox(i).width - 1), (oTrkNd.getTrajBBox(i).y + oTrkNd.getTrajBBox(i).height - 1));

            std::fclose(pfOutTrkTxt);
        }

	}

	if (!oTrkNd.getPredFlg())
    {
        std::sprintf(acOutTrkTxtNm, "%06d.txt", m_nFrmCnt);
        std::sprintf(acOutTrkTxtPth, m_acOutTrkFlrPth);
        std::strcat(acOutTrkTxtPth, acOutTrkTxtNm);

        pfOutTrkTxt = std::fopen(acOutTrkTxtPth, "a");

        std::fprintf(pfOutTrkTxt, "%s 0.0 %d 0.0 %f %f %f %f 0.0 0.0 0.0 0.0 0.0 0.0 0.0\n", oTrkNd.getDetCls(), oTrkNd.getId(),
            oTrkNd.getBBox().x, oTrkNd.getBBox().y, (oTrkNd.getBBox().x + oTrkNd.getBBox().width - 1), (oTrkNd.getBBox().y + oTrkNd.getBBox().height - 1));

        std::fclose(pfOutTrkTxt);
    }
}

void CObjTrk::sortTxtMot(void)
{
	std::vector<CTrkNd> voTrkNd;
	char acBuf[256] = { 0 };
	int nFrmCnt, nId, nDetScr;
	cv::Rect2f oBBox;
	cv::Point3f o3dFtPt;
	FILE* pfOutTrkTxt;

	// read from the txt file
	std::ifstream ifsOutTrkTxt;
	ifsOutTrkTxt.open(m_oCfg.getOutTrkTxtPth());

	ifsOutTrkTxt.getline(acBuf, 256);

	while (!ifsOutTrkTxt.eof())
	{
		std::sscanf(acBuf, "%d,%d,%f,%f,%f,%f,%f,%f,%f,%f", &nFrmCnt, &nId,
			&oBBox.x, &oBBox.y, &oBBox.width, &oBBox.height,
			&nDetScr, &o3dFtPt.x, &o3dFtPt.y, &o3dFtPt.z);

		voTrkNd.push_back(CTrkNd(nFrmCnt, nId, oBBox));

		ifsOutTrkTxt.getline(acBuf, 256);
	}

	ifsOutTrkTxt.close();

	// sort objects by frame counts
	std::sort(voTrkNd.begin(), voTrkNd.end(), cmpFrmCnt);

	// write into the txt file
	pfOutTrkTxt = std::fopen(m_oCfg.getOutTrkTxtPth(), "w");

	for (std::vector<CTrkNd>::iterator it = voTrkNd.begin(); it != voTrkNd.end(); ++it)
	{
		std::fprintf(pfOutTrkTxt, "%d,%d,%.3f,%.3f,%.3f,%.3f,-1,-1,-1,-1\n", it->getFrmCnt(), it->getId(),
			it->getBBox().x, it->getBBox().y, it->getBBox().width, it->getBBox().height);
	}

	std::fclose(pfOutTrkTxt);
}

bool CObjTrk::rdObjTrkMot(std::vector<CTrkNd>& voTrkNd, int nFrmCnt)
{
	bool bNxtFrmFlg = true;

	while ((!m_ifsOutTrkTxt.eof()) && ((nFrmCnt == m_oNxtTrkNd.getFrmCnt()) || (-1 == m_oNxtTrkNd.getFrmCnt())))
	{
		// at the first frame with objects
		if (-1 == m_oNxtTrkNd.getFrmCnt())
			bNxtFrmFlg = false;

		// push back the extra node read from the last iteration
		// not at the first frame with objects
		if (bNxtFrmFlg && (nFrmCnt == m_oNxtTrkNd.getFrmCnt()))
		{
			voTrkNd.push_back(m_oNxtTrkNd);
			bNxtFrmFlg = false;
		}

		char acBuf[256] = { 0 };
		int nTrkFrmCnt, nId;
		float fDetScr = 0.0f;
		cv::Rect2f oBBox;
		cv::Point3f o3dFtPt;

		// read from the input txt file
		m_ifsOutTrkTxt.getline(acBuf, 256);
		if (!m_ifsOutTrkTxt.eof())
        {
            std::sscanf(acBuf, "%d,%d,%f,%f,%f,%f,%f,%f,%f,%f", &nTrkFrmCnt, &nId,
                &oBBox.x, &oBBox.y, &oBBox.width, &oBBox.height,
                &fDetScr, &o3dFtPt.x, &o3dFtPt.y, &o3dFtPt.z);

            if (nTrkFrmCnt >= nFrmCnt)
            {
                // if there is object detected
                if (0 <= nTrkFrmCnt)
                {
                    // validate the detected bounding box
                    if (valBBox(oBBox, m_oCfg.getFrmSz(), m_oImgRoi))
                    {
                        m_oNxtTrkNd = CTrkNd(nTrkFrmCnt, nId, oBBox);

                        // only objects in the current frame are pushed back
                        // filter away detected objects with low score
                        if (nTrkFrmCnt <= nFrmCnt)
                            voTrkNd.push_back(m_oNxtTrkNd);
                    }
                }
            }
        }
	}

	if (m_ifsOutTrkTxt.eof())
        return false;
    else
        return true;
}

bool CObjTrk::rdObjTrkKitti(std::vector<CTrkNd>& voTrkNd, int nFrmCnt)
{
	char acOutTrkTxtNm[128] = { 0 };
	char acOutTrkTxtPth[128] = { 0 };
	std::sprintf(acOutTrkTxtNm, "%06d.txt", nFrmCnt);
	std::sprintf(acOutTrkTxtPth, m_acOutTrkFlrPth);
	std::strcat(acOutTrkTxtPth, acOutTrkTxtNm);

	FILE* pfOutTrkTxt = std::fopen(acOutTrkTxtPth, "r");
    if (pfOutTrkTxt == NULL)
        return false;
	std::fclose(pfOutTrkTxt);

	m_ifsOutTrkTxt.open(acOutTrkTxtPth);

	char acBuf[256] = { 0 };
	int nId;
	float fDummy1, fDummy3, fDummy8, fDummy9, fDummy10, fDummy11, fDummy12, fDummy13, fDummy14;
	cv::Rect2f oBBox;
	cv::Point2f oBBoxMin, oBBoxMax;
	char acDetCls[128];

	while (!m_ifsOutTrkTxt.eof())
	{
		// read from the input txt file
		m_ifsOutTrkTxt.getline(acBuf, 256);
		std::sscanf(acBuf, "%s %f %d %f %f %f %f %f %f %f %f %f %f %f %f", acDetCls, &fDummy1,
			&nId, &fDummy3, &oBBoxMin.x, &oBBoxMin.y, &oBBoxMax.x, &oBBoxMax.y,
			&fDummy8, &fDummy9, &fDummy10, &fDummy11, &fDummy12, &fDummy13, &fDummy14);

		oBBox = cv::Rect2f(oBBoxMin, oBBoxMax);

		if (valBBox(oBBox, m_oCfg.getFrmSz(), m_oImgRoi))
		{
			m_oNxtTrkNd = CTrkNd(nFrmCnt, nId, oBBox);
			voTrkNd.push_back(m_oNxtTrkNd);
		}
	}

	m_ifsOutTrkTxt.close();
	return true;
}

void CObjTrk::pltTrkBBox(cv::Mat& oImgFrm, int nFrmCnt, std::vector<CTrkNd> voTrkNd)
{
	if (voTrkNd.size())
	{
		int nId;
		cv::Rect oBBox;
		char acObjId[8];

		for (std::vector<CTrkNd>::iterator it = voTrkNd.begin(); it != voTrkNd.end(); ++it)
		{
			nId = it->getId();
			oBBox = it->getBBox();

            // plot colored bounding box
            cv::rectangle(oImgFrm, oBBox, m_voBBoxClr[nId % m_voBBoxClr.size()], 2);
            // plot colored object ID
            std::sprintf(acObjId, "%d", it->getId());
            cv::putText(oImgFrm, acObjId, cv::Point((oBBox.x + (oBBox.width / 2)), (oBBox.y + (oBBox.height / 2))), cv::FONT_HERSHEY_SIMPLEX, 1, m_voBBoxClr[nId % m_voBBoxClr.size()], 2);
		}
	}
}
