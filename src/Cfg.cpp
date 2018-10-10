#include "Cfg.h"

CCfg::CCfg()
{
	m_oFrmSz = cv::Size(640, 480);
	m_fFrmRt = 10.0f;
	//std::strcpy(m_acInVdoPth, ".\\data\\vdo.avi");	// in Windows
	std::strcpy(m_acInVdoPth, "./data/vdo.avi");	// in Linux
	//std::strcpy(m_acInFrmFlrPth, ".\\data\\img1\\");	// in Windows
	std::strcpy(m_acInFrmFlrPth, "./data/img1/");	// in Linux
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
	//std::strcpy(m_acOutVdoPth, ".\\data\\outVdo.avi");	// in Windows
	std::strcpy(m_acOutVdoPth, "./data/outVdo.avi");	// in Linux
	//std::strcpy(m_acOutImgFlrPth, ".\\data\\outImg1\\");	// in Windows
	std::strcpy(m_acOutImgFlrPth, "./data/outImg1/");	// in Linux
	m_nInVdoTyp = 0;
	m_nInDetTyp = 0;
	m_nOutTrkTyp = 0;
	m_bProcTrkFlg = true;
	m_bOutVdoFlg = false;
	m_bOutImgFlg = false;
	m_bSelRoiFlg = false;
	m_bPltTrkResFlg = true;
	m_bPltDetFlg = false;
	m_nProcStFrmCnt = 0;
	m_nProcFrmNum = -1;
	m_fOvrdFrmRt = 10.0f;
	m_bPltIdFlg = true;
	m_fPltTrajTmSec = 2.0f;
	m_nRszFrmHei = -1;
	m_fDetScrThld = 30.0f;
        m_fTrkDistRatThld = 0.1f;
	m_fTrkNtrTmSecThld = 0.1f;
	m_fTrkHypTmSecThld = 0.5f;
	m_fTrkTrajTmSecThld = 3.0f;
}

CCfg::~CCfg()
{

}

void CCfg::loadCfgFile()
{
	FILE * poCfgFile;
	long nlFileSz, nlRdRst;
	char * pcBuf;

	//poCfgFile = std::fopen(".\\data\\cfg.json", "r");	// in Windows
	poCfgFile = std::fopen("./data/cfg.json", "r");	// in Linux
	if (poCfgFile == NULL) { fputs("Error: configuration file not opened", stderr); exit(1); }

	// obtain file size:
	fseek(poCfgFile, 0, SEEK_END);
	nlFileSz = ftell(poCfgFile);
	rewind(poCfgFile);

	// allocate memory to contain the whole file:
	pcBuf = (char*)malloc(sizeof(char)*nlFileSz);
	if (pcBuf == NULL) { fputs("Memory error", stderr); exit(2); }

	// copy the file into the buffer:
	nlRdRst = fread(pcBuf, 1, nlFileSz, poCfgFile);
	//if (nlRdRst != nlFileSz) { fputs("Reading error", stderr); exit(3); }

	std::string strCfg(pcBuf);
	//strCfg.erase(std::remove_if(strCfg.begin(), strCfg.end(), [](char c) { return c >= 0 && isspace(c); }), strCfg.end());	// in Windows
        strCfg.erase(std::remove_if(strCfg.begin(), strCfg.end(), ::isspace), strCfg.end());	// in Linux

	int nParamPos = strCfg.find("\"inVdoPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acInVdoPth, readCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"inFrmFlrPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acInFrmFlrPth, readCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"inRoiPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acInRoiPth, readCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"inDetTxtPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acInDetTxtPth, readCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"inDetFlrPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acInDetFlrPth, readCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"outTrkTxtPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acOutTrkTxtPth, readCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"outTrkFlrPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acOutTrkFlrPth, readCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"outVdoPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acOutVdoPth, readCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"outImgFlrPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acOutImgFlrPth, readCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"inVdoTyp\"");
	if (nParamPos != std::string::npos)
		m_nInVdoTyp = readInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"inDetTyp\"");
	if (nParamPos != std::string::npos)
		m_nInDetTyp = readInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"outTrkTyp\"");
	if (nParamPos != std::string::npos)
		m_nOutTrkTyp = readInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"procTrkFlg\"");
	if (nParamPos != std::string::npos)
		m_bProcTrkFlg = readBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"outVdoFlg\"");
	if (nParamPos != std::string::npos)
		m_bOutVdoFlg = readBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"outImgFlg\"");
	if (nParamPos != std::string::npos)
		m_bOutImgFlg = readBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"selRoiFlg\"");
	if (nParamPos != std::string::npos)
		m_bSelRoiFlg = readBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"pltTrkResFlg\"");
	if (nParamPos != std::string::npos)
		m_bPltTrkResFlg = readBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"pltDetFlg\"");
	if (nParamPos != std::string::npos)
		m_bPltDetFlg = readBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"procStFrmCnt\"");
	if (nParamPos != std::string::npos)
		m_nProcStFrmCnt = readInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"procFrmNum\"");
	if (nParamPos != std::string::npos)
		m_nProcFrmNum = readInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"ovrdFrmRt\"");
	if (nParamPos != std::string::npos)
		m_fOvrdFrmRt = readFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"pltIdFlg\"");
	if (nParamPos != std::string::npos)
		m_bPltIdFlg = readBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"pltTrajTmSec\"");
	if (nParamPos != std::string::npos)
		m_fPltTrajTmSec = readFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"rszFrmHei\"");
	if (nParamPos != std::string::npos)
		m_nRszFrmHei = readInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"detScrThld\"");
	if (nParamPos != std::string::npos)
		m_fDetScrThld = readFlt(strCfg, nParamPos);

        nParamPos = strCfg.find("\"trkDistRatThld\"");
	if (nParamPos != std::string::npos)
		m_fTrkDistRatThld = readFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"trkNtrTmSecThld\"");
	if (nParamPos != std::string::npos)
		m_fTrkNtrTmSecThld = readFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"trkHypTmSecThld\"");
	if (nParamPos != std::string::npos)
		m_fTrkHypTmSecThld = readFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"trkTrajTmSecThld\"");
	if (nParamPos != std::string::npos)
		m_fTrkTrajTmSecThld = readFlt(strCfg, nParamPos);

	// terminate
	fclose(poCfgFile);
	free(pcBuf);
}

std::string CCfg::readCharArr(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 2;
	nValLen = strCfg.find("\"", (nValPos + 1)) - nValPos;

	return strCfg.substr(nValPos, nValLen);
}

int CCfg::readInt(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd1, nValEnd2;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd1 = strCfg.find(",", (nValPos + 1));
	nValEnd2 = strCfg.find("}", (nValPos + 1));
	nValLen = (nValEnd1 <= nValEnd2) ? (nValEnd1 - nValPos) : (nValEnd2 - nValPos);

	return std::atoi(strCfg.substr(nValPos, nValLen).c_str());
}

float CCfg::readFlt(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd1, nValEnd2;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd1 = strCfg.find(",", (nValPos + 1));
	nValEnd2 = strCfg.find("}", (nValPos + 1));
	nValLen = (nValEnd1 <= nValEnd2) ? (nValEnd1 - nValPos) : (nValEnd2 - nValPos);

	return std::atof(strCfg.substr(nValPos, nValLen).c_str());
}

bool CCfg::readBool(std::string strCfg, int nParamPos)
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
