#include "ObjDet.h"

CDetNd::CDetNd(void)
{
	setFrmCnt(-1);
	setDetScr(0.0f);
}

CDetNd::CDetNd(cv::Rect2f oDetBBox, float fDetScr, char* acDetCls, int nFrmCnt)
{
	setDetBBox(oDetBBox);
	setDetScr(fDetScr);
	setDetCls(acDetCls);
	setFrmCnt(nFrmCnt);
}

CDetNd::~CDetNd(void)
{

}

CObjDet::CObjDet(void)
{

}

CObjDet::~CObjDet(void)
{
	if (0 == m_oCfg.getInDetTyp())
		m_ifsInDetTxt.close();
}

void CObjDet::initialize(CCfg oCfg, cv::Mat oImgRoi)
{
	// configuration parameters
	m_oCfg = oCfg;

	// ROI image
	m_oImgRoi = oImgRoi.clone();

	// next detected node
	m_oNxtDetNd = CDetNd();

	// text file for reading detection results
	if (0 == m_oCfg.getInDetTyp())
	{
		m_ifsInDetTxt.close();
		m_ifsInDetTxt.open(m_oCfg.getInDetTxtPth());

		FILE* pfInDetTxt = std::fopen(m_oCfg.getInDetTxtPth(), "r");
		if (pfInDetTxt == NULL)
		{
			std::cout << "Error: The KITTI detection results are not available. " << std::endl;
			exit(1);
		}
		std::fclose(pfInDetTxt);
	}
    // folder of detection results
	else if (1 == m_oCfg.getInDetTyp())
	{
		char acInDetTxtNm[128] = { 0 };
		char acInDetTxtPth[128] = { 0 };
		std::sprintf(acInDetTxtNm, "%06d.txt", m_oCfg.getProcStFrmCnt());
		std::sprintf(m_acInDetFlrPth, oCfg.getInDetFlrPth());
		std::sprintf(acInDetTxtPth, m_acInDetFlrPth);
		std::strcat(acInDetTxtPth, acInDetTxtNm);

		FILE * pfInDetTxt = std::fopen(acInDetTxtPth, "r");
		if (pfInDetTxt == NULL)
		{
			std::cout << "Error: The KITTI detection results are not available. " << std::endl;
			exit(1);
		}
		std::fclose(pfInDetTxt);
	}
}

bool CObjDet::process(std::vector<CDetNd>& voDetNd, int nFrmCnt)
{
    bool bProcFlg;
	std::vector<CDetNd>().swap(voDetNd);

	// MOTChallenge format
	if (0 == m_oCfg.getInDetTyp())
		bProcFlg = rdObjDetMot(voDetNd, nFrmCnt);
	// KITTI format
	else if (1 == m_oCfg.getInDetTyp())
		bProcFlg = rdObjDetKitti(voDetNd, nFrmCnt);

	if (NMS_DET_FLG)
		nonMaxSuppr(voDetNd);

	rmvOutBBox(voDetNd);

	return bProcFlg;
}

bool CObjDet::rdObjDetMot(std::vector<CDetNd>& voDetNd, int nFrmCnt)
{
	bool bNxtFrmFlg = true;

	while ((!m_ifsInDetTxt.eof()) && ((nFrmCnt == m_oNxtDetNd.getFrmCnt()) || (-1 == m_oNxtDetNd.getFrmCnt())))
	{
		// at the first frame with objects
		if (-1 == m_oNxtDetNd.getFrmCnt())
			bNxtFrmFlg = false;

		// push back the extra node read from the last iteration
		// not at the first frame with objects
		if (bNxtFrmFlg && (nFrmCnt == m_oNxtDetNd.getFrmCnt()))
		{
			voDetNd.push_back(m_oNxtDetNd);
			bNxtFrmFlg = false;
		}

		char acInDetBuf[256] = { 0 };
		int nDetFrmCnt, nDetId;
		float fDetScr;
		cv::Rect2f oDetBBox;
		cv::Point3f oDet3dFtPt;
		char acDetCls[128] = { 0 };

		// read from the input txt file
		m_ifsInDetTxt.getline(acInDetBuf, 256);
		if (!m_ifsInDetTxt.eof())
		{
		    std::sscanf(acInDetBuf, "%d,%d,%f,%f,%f,%f,%f,%f,%f,%f", &nDetFrmCnt, &nDetId,
			&oDetBBox.x, &oDetBBox.y, &oDetBBox.width, &oDetBBox.height,
			&fDetScr, &oDet3dFtPt.x, &oDet3dFtPt.y, &oDet3dFtPt.z);

		    if (nDetFrmCnt >= nFrmCnt)
		    {
			// if there is object detected
			if (0 <= nDetFrmCnt)
			{
			    // validate the detected bounding box
			    if (valBBox(oDetBBox, m_oCfg.getFrmSz(), m_oImgRoi))
			    {
				m_oNxtDetNd = CDetNd(oDetBBox, fDetScr, acDetCls, nDetFrmCnt);

				// only objects in the current frame are pushed back
				if (nDetFrmCnt == nFrmCnt)
				{
				    // filter away detected objects with low score
				    if (m_oCfg.getDetScrThld() <= fDetScr)
					voDetNd.push_back(m_oNxtDetNd);
				}
			    }
			}
		    }
		}
	}

	if (!m_ifsInDetTxt.eof())
        return true;
    else
        return false;
}

bool CObjDet::rdObjDetKitti(std::vector<CDetNd>& voDetNd, int nFrmCnt)
{
	char acInDetTxtNm[128] = { 0 };
	char acInDetTxtPth[128] = { 0 };
	std::sprintf(acInDetTxtNm, "%06d.txt", nFrmCnt);
	std::sprintf(acInDetTxtPth, m_acInDetFlrPth);
	std::strcat(acInDetTxtPth, acInDetTxtNm);

	FILE* pfInDetTxt = std::fopen(acInDetTxtPth, "r");
    if (pfInDetTxt == NULL)
        return false;
	std::fclose(pfInDetTxt);

	m_ifsInDetTxt.open(acInDetTxtPth);

	char acInDetBuf[256] = { 0 };
	int nDummy2;
	float fDummy1, fDummy3, fDummy8, fDummy9, fDummy10, fDummy11, fDummy12, fDummy13, fDummy14;
	cv::Rect2f oDetBBox;
	cv::Point2f oDetBBoxMin, oDetBBoxMax;
	char acDetCls[128];

	while (!m_ifsInDetTxt.eof())
	{
		// read from the input txt file
		m_ifsInDetTxt.getline(acInDetBuf, 256);
		std::sscanf(acInDetBuf, "%s %f %d %f %f %f %f %f %f %f %f %f %f %f %f", acDetCls, &fDummy1,
			&nDummy2, &fDummy3, &oDetBBoxMin.x, &oDetBBoxMin.y, &oDetBBoxMax.x, &oDetBBoxMax.y,
			&fDummy8, &fDummy9, &fDummy10, &fDummy11, &fDummy12, &fDummy13, &fDummy14);

		oDetBBox = cv::Rect2f(oDetBBoxMin, oDetBBoxMax);

		if (valBBox(oDetBBox, m_oCfg.getFrmSz(), m_oImgRoi))
		{
			m_oNxtDetNd = CDetNd(oDetBBox, m_oCfg.getDetScrThld(), acDetCls, nFrmCnt);
			voDetNd.push_back(m_oNxtDetNd);
		}
	}

	m_ifsInDetTxt.close();
	return true;
}

void CObjDet::nonMaxSuppr(std::vector<CDetNd>& voDetNd)
{
	std::vector<int> viNonMaxDetNd;
	std::vector<int>::iterator it;

	for (int i = voDetNd.size() - 1; i >= 0; i--)
	{
		// a pair of bounding boxes overlap with each other: erase the one with lower detection score
		for (int j = i - 1; j >= 0; j--)
		{
			if (NMS_DET_IOU_THLD < calcBBoxIou(voDetNd[i].getDetBBox(), voDetNd[j].getDetBBox()))
			{
				int iNonMaxDetNd = (voDetNd[i].getDetScr() < voDetNd[j].getDetScr()) ? i : j;
				it = find(viNonMaxDetNd.begin(), viNonMaxDetNd.end(), iNonMaxDetNd);
				if (viNonMaxDetNd.end() == it)
					viNonMaxDetNd.push_back(iNonMaxDetNd);
			}
		}
	}

	for (int i = voDetNd.size() - 1; i >= 0; i--)
	{
		it = find(viNonMaxDetNd.begin(), viNonMaxDetNd.end(), i);
		if (viNonMaxDetNd.end() != it)
		{
			voDetNd.erase(voDetNd.begin() + i);
			viNonMaxDetNd.erase(it);
		}
	}
}

void CObjDet::rmvOutBBox(std::vector<CDetNd>& voDetNd)
{
	std::vector<int> viOutDetNd;
	std::vector<int>::iterator it;

        // remove large bounding box(es) that contain other bounding box(es)
	for (int i = voDetNd.size() - 1; i >= 0; i--)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			if ((voDetNd[i].getDetBBox().x <= voDetNd[j].getDetBBox().x) &&
				(voDetNd[i].getDetBBox().y <= voDetNd[j].getDetBBox().y) &&
				((voDetNd[i].getDetBBox().x + voDetNd[i].getDetBBox().width) >=
					(voDetNd[j].getDetBBox().x + voDetNd[j].getDetBBox().width)) &&
				((voDetNd[i].getDetBBox().y + voDetNd[i].getDetBBox().height) >=
					(voDetNd[j].getDetBBox().y + voDetNd[j].getDetBBox().height)))
			{
				it = find(viOutDetNd.begin(), viOutDetNd.end(), i);
				if (viOutDetNd.end() == it)
				{
					if (0 == std::strcmp(voDetNd[i].getDetCls(), voDetNd[j].getDetCls()))
						viOutDetNd.push_back(i);
				}
				break;
			}
			else if ((voDetNd[j].getDetBBox().x <= voDetNd[i].getDetBBox().x) &&
				(voDetNd[j].getDetBBox().y <= voDetNd[i].getDetBBox().y) &&
				((voDetNd[j].getDetBBox().x + voDetNd[j].getDetBBox().width) >=
					(voDetNd[i].getDetBBox().x + voDetNd[i].getDetBBox().width)) &&
				((voDetNd[j].getDetBBox().y + voDetNd[j].getDetBBox().height) >=
					(voDetNd[i].getDetBBox().y + voDetNd[i].getDetBBox().height)))
			{
				it = find(viOutDetNd.begin(), viOutDetNd.end(), j);
				if (viOutDetNd.end() == it)
				{
					if (0 == std::strcmp(voDetNd[j].getDetCls(), voDetNd[i].getDetCls()))
						viOutDetNd.push_back(j);
				}
				break;
			}
		}
	}

	for (int i = voDetNd.size() - 1; i >= 0; i--)
	{
		it = find(viOutDetNd.begin(), viOutDetNd.end(), i);
		if (viOutDetNd.end() != it)
		{
			voDetNd.erase(voDetNd.begin() + i);
			viOutDetNd.erase(it);
		}
	}
}
