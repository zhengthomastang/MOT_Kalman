#pragma once

#include <iostream>
#include <opencv2/core/core.hpp>

class CCfg
{
public:
	//! full constructor
	CCfg();
	//! default destructor
	~CCfg();

	//! loads configuration file from directory
	void ldCfgFl(char* acCfgFlPth);

	inline cv::Size getFrmSz(void) { return m_oFrmSz; }
	void setFrmSz(cv::Size oFrmSz) { m_oFrmSz = oFrmSz; };
	inline float getFrmRt(void) { return m_fFrmRt; }
	inline void setFrmRt(float fFrmRt) { m_fFrmRt = fFrmRt; }
	inline int getRoiArea(void) { return m_nRoiArea; }
	inline void setRoiArea(int nRoiArea) { m_nRoiArea = nRoiArea; }

	inline char* getInFrmFlrPth(void) { return m_acInFrmFlrPth; }
	inline char* getInVdoPth(void) { return m_acInVdoPth; }
	inline char* getInRoiPth(void) { return m_acInRoiPth; }
	inline char* getInDetTxtPth(void) { return m_acInDetTxtPth; }
	inline char* getInDetFlrPth(void) { return m_acInDetFlrPth; }
	inline char* getOutTrkTxtPth(void) { return m_acOutTrkTxtPth; }
	inline char* getOutTrkFlrPth(void) { return m_acOutTrkFlrPth; }
	inline char* getOutImgFlrPth(void) { return m_acOutImgFlrPth; }
	inline int getInVdoTyp(void) { return m_nInVdoTyp; }
	inline int getInDetTyp(void) { return m_nInDetTyp; }
	inline int getOutTrkTyp(void) { return m_nOutTrkTyp; }
	inline bool getOutImgFlg(void) { return m_bOutImgFlg; }
	inline bool getSelRoiFlg(void) { return m_bSelRoiFlg; }
	inline int getProcStFrmCnt(void) { return m_nProcStFrmCnt; }
	inline int getProcFrmNum(void) { return m_nProcFrmNum; }
	inline cv::Size getOvrdFrmSz(void) { return m_oOvrdFrmSz; }
	inline double getOvrdFrmRt(void) { return m_fOvrdFrmRt; }
    inline bool getPltIdFlg(void) { return m_bPltIdFlg; }
	inline int getRszFrmHei(void) { return m_nRszFrmHei; }
	inline float getDetScrThld(void) { return m_fDetScrThld; }
    inline float getTrkDistRatThld(void) { return m_fTrkDistRatThld; }
	inline int getTrkDistThld(void) { return m_fTrkDistRatThld * (m_oFrmSz.width + m_oFrmSz.height) / 2; }
	inline float getTrkNtrTmSecThld(void) { return m_fTrkNtrTmSecThld; }
	inline int getTrkNtrFrmNumThld(void) { return m_fTrkNtrTmSecThld * m_fFrmRt; }
	inline float getTrkHypTmSecThld(void) { return m_fTrkHypTmSecThld; }
	inline int getTrkHypFrmNumThld(void) { return m_fTrkHypTmSecThld * m_fFrmRt; }

private:
	//! reads char array
	std::string rdCharArr(std::string strCfg, int nParamPos);
	//! reads integer number
	int rdInt(std::string strCfg, int nParamPos);
	//! reads float number
	float rdFlt(std::string strCfg, int nParamPos);
	//! reads bool value
	bool rdBool(std::string strCfg, int nParamPos);
	//! reads size
	cv::Size rdSz(std::string strCfg, int nParamPos);

	//! video frame size
	cv::Size m_oFrmSz;
	//! video frame rate
	float m_fFrmRt;
	//! ROI area
	int m_nRoiArea;
	//! path of folder for input image files, necessary when m_nInVdoTyp == 1
	char m_acInFrmFlrPth[256];
	//! path of input video stream, necessary when m_nInVdoTyp == 2
	char m_acInVdoPth[256];
	//! path of input ROI image
	char m_acInRoiPth[256];
	//! path of input text file of object detection, necessary when m_nInDetTyp == 0
	char m_acInDetTxtPth[256];
	//! path of input folder of object detection, necessary when m_nInDetTyp == 1
	char m_acInDetFlrPth[256];
	//! path of output text file of tracking results, necessary when m_nOutTrkTyp == 0
	char m_acOutTrkTxtPth[256];
	//! path of output folder of tracking results, necessary when m_nOutTrkTyp == 1
	char m_acOutTrkFlrPth[256];
	//! path of folder for output image files, necessary when m_bOutImgFlg == true
	char m_acOutImgFlrPth[256];
	//! type of input video source: 0: no video source; 1: image files; 2: video file (1 or 2 when m_bOutImgFlg == 1)
	int m_nInVdoTyp;
	//! type of input detection: 0: MOTChallenge format; 1: KITTI format
	int m_nInDetTyp;
	//! type of output tracking: 0: MOTChallenge format; 1: KITTI format
	int m_nOutTrkTyp;
	//! flag of output image files
	bool m_bOutImgFlg;
	//! flag of selecting ROI image
	bool m_bSelRoiFlg;
	//! starting frame count to process
	int m_nProcStFrmCnt;
	//! number of frames to process (-1: till the end of the video source)
	int m_nProcFrmNum;
	 //! overriden frame size, necessary when m_nInVdoTyp == 0
    cv::Size m_oOvrdFrmSz;
	//! overriden frame rate, necessary when m_nInVdoTyp == 0 or m_nInVdoTyp == 1
	double m_fOvrdFrmRt;
    //! flag of plotting the number of object identity, necessary when m_nOutTrkTyp == 0 and m_bOutImgFlg == true
    bool m_bPltIdFlg;
	//! resized video frame height (-1: original size)
	int m_nRszFrmHei;
	//! threshold of detection score (in percentage)
	float m_fDetScrThld;
    //! threshold for distance in terms of ratio of frame size to determine nearby object nodes
    float m_fTrkDistRatThld;
	//! threshold of existing time in seconds for entering objects
	float m_fTrkNtrTmSecThld;
	//! threshold of time in seconds that the objects are tracked as hypotheses (by prediction)
	float m_fTrkHypTmSecThld;
};
