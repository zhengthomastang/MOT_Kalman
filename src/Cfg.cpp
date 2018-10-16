#include "Cfg.h"

CCfg::CCfg()
{
	m_oFrmSz = cv::Size(640, 480);
	m_fFrmRt = 10.0f;
	//std::strcpy(m_acInFrmFlrPth, ".\\data\\img1\\");	// in Windows
	std::strcpy(m_acInFrmFlrPth, "./data/img1/");	// in Linux
	//std::strcpy(m_acInVdoPth, ".\\data\\vdo.avi");	// in Windows
	std::strcpy(m_acInVdoPth, "./data/vdo.avi");	// in Linux
	//std::strcpy(m_acInRoiPth, ".\\data\\roi.jpg");	// in Windows
	std::strcpy(m_acInRoiPth, "./data/roi.jpg");	// in Linux
	//std::strcpy(m_acInDetTxtPth, ".\\data\\det.txt");	// in Windows
	std::strcpy(m_acInDetTxtPth, "./data/det.txt");	// in Linux
	//std::strcpy(m_acInDetFlrPth, ".\\data\\det\\");	// in Windows
	std::strcpy(m_acInDetFlrPth, "./data/det/");	// in Linux
	//std::strcpy(m_acOutTrkTxtPth, ".\\data\\res.txt");	// in Windows
	std::strcpy(m_acOutTrkTxtPth, "./data/res.txt");	// in Linux
	//std::strcpy(m_acOutTrkFlrPth, ".\\data\\res\\");	// in Windows
	std::strcpy(m_acOutTrkFlrPth, "./data/res/");	// in Linux
	//std::strcpy(m_acOutImgFlrPth, ".\\data\\outImg1\\");	// in Windows
	std::strcpy(m_acOutImgFlrPth, "./data/outImg1/");	// in Linux
	//std::strcpy(m_acOutVdoPth, ".\\data\\outVdo.avi");	// in Windows
	std::strcpy(m_acOutVdoPth, "./data/outVdo.avi");	// in Linux
	m_nInVdoTyp = 0;
	m_nInDetTyp = 0;
	m_nOutTrkTyp = 0;
	m_nOutVdoTyp = 0;
	m_bSelRoiFlg = false;
	m_nProcStFrmCnt = 0;
	m_nProcFrmNum = -1;
	m_oOvrdFrmSz = m_oFrmSz;
	m_fOvrdFrmRt = 10.0f;
	m_bPltIdFlg = true;
	m_nRszFrmHei = -1;
	m_fDetScrThld = 30.0f;
    m_fTrkFrmRtThld = 5.0f;
	m_fTrkMtchScrThld = 0.35f;
	m_fTrkNtrScrThld = 0.25f;
	m_fTrkNtrTmSecThld = 0.1f;
	m_fTrkPredTmSecThld = 0.5f;
}

CCfg::~CCfg()
{

}

void CCfg::ldCfgFl(char* acCfgFlPth)
{
	FILE * poCfgFl;
	long nlFileSz, nlRdRst;
	char * pcBuf;

    if (NULL == acCfgFlPth)
		//poCfgFl = std::fopen(".\\data\\cfg.json", "r");	// in Windows
		poCfgFl = std::fopen("./data/cfg.json", "r");	// in Linux
	else
		poCfgFl = std::fopen(acCfgFlPth, "r");

	// obtain file size:
	fseek(poCfgFl, 0, SEEK_END);
	nlFileSz = ftell(poCfgFl);
	rewind(poCfgFl);

	// allocate memory to contain the whole file:
	pcBuf = (char*)malloc(sizeof(char)*nlFileSz);
	if (pcBuf == NULL) { fputs("Memory error", stderr); exit(2); }

	// copy the file into the buffer:
	nlRdRst = fread(pcBuf, 1, nlFileSz, poCfgFl);
	//if (nlRdRst != nlFileSz) { fputs("Reading error", stderr); exit(3); }

	std::string strCfg(pcBuf);
	//strCfg.erase(std::remove_if(strCfg.begin(), strCfg.end(), [](char c) { return c >= 0 && isspace(c); }), strCfg.end());	// in Windows
    strCfg.erase(std::remove_if(strCfg.begin(), strCfg.end(), ::isspace), strCfg.end());	// in Linux

	int nParamPos = strCfg.find("\"inFrmFlrPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acInFrmFlrPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"inVdoPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acInVdoPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"inRoiPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acInRoiPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"inDetTxtPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acInDetTxtPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"inDetFlrPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acInDetFlrPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"outTrkTxtPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acOutTrkTxtPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"outTrkFlrPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acOutTrkFlrPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"outImgFlrPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acOutImgFlrPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"outVdoPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acOutVdoPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"inVdoTyp\"");
	if (nParamPos != std::string::npos)
		m_nInVdoTyp = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"inDetTyp\"");
	if (nParamPos != std::string::npos)
		m_nInDetTyp = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"outTrkTyp\"");
	if (nParamPos != std::string::npos)
		m_nOutTrkTyp = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"outVdoTyp\"");
	if (nParamPos != std::string::npos)
		m_nOutVdoTyp = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"selRoiFlg\"");
	if (nParamPos != std::string::npos)
		m_bSelRoiFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"procStFrmCnt\"");
	if (nParamPos != std::string::npos)
		m_nProcStFrmCnt = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"procFrmNum\"");
	if (nParamPos != std::string::npos)
		m_nProcFrmNum = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"ovrdFrmSz\"");
	if (nParamPos != std::string::npos)
		m_oOvrdFrmSz = rdSz(strCfg, nParamPos);

	nParamPos = strCfg.find("\"ovrdFrmRt\"");
	if (nParamPos != std::string::npos)
		m_fOvrdFrmRt = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"pltIdFlg\"");
	if (nParamPos != std::string::npos)
		m_bPltIdFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"rszFrmHei\"");
	if (nParamPos != std::string::npos)
		m_nRszFrmHei = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"detScrThld\"");
	if (nParamPos != std::string::npos)
		m_fDetScrThld = rdFlt(strCfg, nParamPos);

    nParamPos = strCfg.find("\"trkFrmRtThld\"");
	if (nParamPos != std::string::npos)
		m_fTrkFrmRtThld = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"trkMtchScrThld\"");
	if (nParamPos != std::string::npos)
		m_fTrkMtchScrThld = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"trkNtrScrThld\"");
	if (nParamPos != std::string::npos)
		m_fTrkNtrScrThld = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"trkNtrTmSecThld\"");
	if (nParamPos != std::string::npos)
		m_fTrkNtrTmSecThld = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"trkPredTmSecThld\"");
	if (nParamPos != std::string::npos)
		m_fTrkPredTmSecThld = rdFlt(strCfg, nParamPos);

	CV_Assert((0 == m_nInVdoTyp) || (1 == m_nInVdoTyp) || (2 == m_nInVdoTyp));
	CV_Assert((0 == m_nInDetTyp) || (1 == m_nInDetTyp));
	CV_Assert((0 == m_nOutTrkTyp) || (1 == m_nOutTrkTyp));
	if (0 == m_nInVdoTyp)
		CV_Assert((0 < m_oOvrdFrmSz.width) && (0 < m_oOvrdFrmSz.height));
	if (((0 == m_nInVdoTyp) || (1 == m_nInVdoTyp)))
		CV_Assert(0 < m_fOvrdFrmRt);
	CV_Assert(0 <= m_fDetScrThld);
	if (((1 == m_nOutVdoTyp) || (2 == m_nOutVdoTyp)))
		CV_Assert((1 == m_nInVdoTyp) || (2 == m_nInVdoTyp));
	CV_Assert(0 < m_fTrkFrmRtThld);
	CV_Assert(0 < m_fTrkMtchScrThld);
	CV_Assert(0 < m_fTrkNtrScrThld);
	CV_Assert(0 <= m_fTrkNtrTmSecThld);
	CV_Assert(0 <= m_fTrkPredTmSecThld);

	// terminate
	fclose(poCfgFl);
	free(pcBuf);
}

std::string CCfg::rdCharArr(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 2;
	nValLen = strCfg.find("\"", (nValPos + 1)) - nValPos;

	return strCfg.substr(nValPos, nValLen);
}

int CCfg::rdInt(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd1, nValEnd2;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd1 = strCfg.find(",", (nValPos + 1));
	nValEnd2 = strCfg.find("}", (nValPos + 1));
	nValLen = (nValEnd1 <= nValEnd2) ? (nValEnd1 - nValPos) : (nValEnd2 - nValPos);

	return std::atoi(strCfg.substr(nValPos, nValLen).c_str());
}

float CCfg::rdFlt(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd1, nValEnd2;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd1 = strCfg.find(",", (nValPos + 1));
	nValEnd2 = strCfg.find("}", (nValPos + 1));
	nValLen = (nValEnd1 <= nValEnd2) ? (nValEnd1 - nValPos) : (nValEnd2 - nValPos);

	return std::atof(strCfg.substr(nValPos, nValLen).c_str());
}

bool CCfg::rdBool(std::string strCfg, int nParamPos)
{
	int nBoolVal, nValPos, nValLen, nValEnd1, nValEnd2;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd1 = strCfg.find(",", (nValPos + 1));
	nValEnd2 = strCfg.find("}", (nValPos + 1));
	nValLen = (nValEnd1 <= nValEnd2) ? (nValEnd1 - nValPos) : (nValEnd2 - nValPos);

	nBoolVal = std::atoi(strCfg.substr(nValPos, nValLen).c_str());
	if (nBoolVal > 0)
		return true;
	else if (nBoolVal <= 0)
		return false;
}

cv::Size CCfg::rdSz(std::string strCfg, int nParamPos)
{
	int nWidPos = nParamPos, nWidLen, nHeiPos, nHeiLen;

	nWidPos = strCfg.find(":", (nParamPos + 1)) + 2;
	nWidLen = strCfg.find(",", (nWidPos + 1)) - nWidPos;
	nHeiPos = nWidPos + nWidLen + 1;
	nHeiLen = strCfg.find("]", (nHeiPos + 1)) - nHeiPos;
	cv::Size oSz(std::atof(strCfg.substr(nWidPos, nWidLen).c_str()), std::atof(strCfg.substr(nHeiPos, nHeiLen).c_str()));

	return oSz;
}
