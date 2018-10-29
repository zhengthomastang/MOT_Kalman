#pragma once

//#include <direct.h>	// in Windows
#include <sys/stat.h>	// in Linux
#include <opencv2/video/tracking.hpp> // for Kalman filter
#include "utils.h"
#include "Cfg.h"
#include "ObjDet.h"

//! define the default state of an object node
#define ND_ST8_DFLT (-1)
//! define the normal state of an object node
#define ND_ST8_NORM (0)
//! define the occluded state of an object node
#define ND_ST8_OCCL (1)
//! define the edge state of an object node
#define ND_ST8_EDG (2)
//! define the threshold for the IOU between two tracking nodes that indicates full overlap (default: 0.85f)
#define FULL_OVLP_IOU_THLD (0.85f)
//! define the threshold for visibility to determine the occluded-by-object state (default: 0.50f)
#define VIS_OCCL_OBJ_THLD (0.50f)
//! define the values for foot points coordinates in the process noise covariance matrix (Q) of Kalman filter (default: 1e-1)
#define KF_PROC_NOISE_COV_FTPT (1e-1)
//! define the values for velocities in the process noise covariance matrix (Q) of Kalman filter (default: 1e-1)
#define KF_PROC_NOISE_COV_VEL (1e-1)
//! define the values for sizes of bounding boxes in the process noise covariance matrix (Q) of Kalman filter (default: 1e-1)
#define KF_PROC_NOISE_COV_BBOX_SZ (1e-1)
//! define the values in the measurement noise covariance matrix (R) of Kalman filter (default: 1e-1)
#define KF_MEAS_NOISE_COV (1e-1)
//! define the values in the priori error estimate covariance matrix (P'(k)) of Kalman filter (default: 1)
#define KF_ERR_COV_PRE (1)
//! define the breadth in pixels to determine frame/ROI edge case (default: 10)
#define FRM_EDG_BDTH (10)
//! define the starting object ID (default: 1)
#define ST_OBJ_ID (1)
//! define the starting frame count (default: 1)
#define ST_FRM_CNT (1)

// trajectory of tracking in terms of frame counts, bounding boxes, 2D foot points
class CTraj
{
public:
	CTraj(void);
	CTraj(std::vector<int> vnFrmCnt, std::vector<cv::Rect2f> voBBox, std::vector<cv::Point2f> vo2dFtPt, std::vector<bool> vbPredFlg);
	~CTraj(void);

	inline int getTrajLen(void) { return m_vnTrajFrmCnt.size(); }
	inline void addTrajNd(int nFrmCnt, cv::Rect2f oBBox, cv::Point2f o2dFtPt, bool bPredFlg)
	{
		addTrajFrmCnt(nFrmCnt);
		addTrajBBox(oBBox);
		addTraj2dFtPt(o2dFtPt);
		addTrajPredFlg(bPredFlg);
	}
	inline std::vector<int> getTrajFrmCnts() { return m_vnTrajFrmCnt; }
	inline int getTrajFrmCnt(int nIdx) { return m_vnTrajFrmCnt[nIdx]; }
	inline void setTrajFrmCnts(std::vector<int> vnFrmCnt) { m_vnTrajFrmCnt = vnFrmCnt; }
	inline void addTrajFrmCnt(int nFrmCnt) { m_vnTrajFrmCnt.push_back(nFrmCnt); }
	inline std::vector<cv::Rect2f> getTrajBBoxs() { return m_voTrajBBox; }
	inline cv::Rect2f getTrajBBox(int nIdx) { return m_voTrajBBox[nIdx]; }
	inline void setTrajBBoxs(std::vector<cv::Rect2f> voBBox) { m_voTrajBBox = voBBox; }
	inline void addTrajBBox(cv::Rect2f oBBox) { m_voTrajBBox.push_back(oBBox); }
	inline std::vector<cv::Point2f> getTraj2dFtPts() { return m_voTraj2dFtPt; }
	inline cv::Point2f getTraj2dFtPt(int nIdx) { return m_voTraj2dFtPt[nIdx]; }
	inline void setTraj2dFtPts(std::vector<cv::Point2f> vo2dFtPt) { m_voTraj2dFtPt = vo2dFtPt; }
	inline void addTraj2dFtPt(cv::Point2f o2dFtPt) { m_voTraj2dFtPt.push_back(o2dFtPt); }
	inline std::vector<bool> getTrajPredFlgs() { return m_vbTrajPredFlg; }
	inline bool getTrajPredFlg(int nIdx) { return m_vbTrajPredFlg[nIdx]; }
	inline void setTrajPredFlgs(std::vector<bool> vbTrajPredFlg) { m_vbTrajPredFlg = vbTrajPredFlg; }
	inline void addTrajPredFlg(bool bPredFlg) { m_vbTrajPredFlg.push_back(bPredFlg); }
	inline void resetTrajPredFlg(void)
	{
	    for (int i = m_vbTrajPredFlg.size() - 1; i >= 0; i--)
        {
            if (m_vbTrajPredFlg[i])
                m_vbTrajPredFlg[i] = false;
        }
    }
	inline int getTrajPredNum(void)
	{
	    int nTrajPredNum = 0;
	    for (int i = m_vbTrajPredFlg.size() - 1; i >= 0; i--)
        {
            if (m_vbTrajPredFlg[i])
                nTrajPredNum++;
            else
                break;
        }
	    return nTrajPredNum;
    }

protected:
	//! frame counts
	std::vector<int> m_vnTrajFrmCnt;
	//! object bounding boxes
	std::vector<cv::Rect2f> m_voTrajBBox;
	//! 2D foot points
	std::vector<cv::Point2f> m_voTraj2dFtPt;
	//! flags of tracking by prediction
	std::vector<bool> m_vbTrajPredFlg;
};

// candidate node for tracker to match with
class CCandNd : public CDetNd
{
public:
	CCandNd(void);
	CCandNd(CDetNd oDetNd, int nSt8);
	~CCandNd(void);

	inline void setDetNd(CDetNd oDetNd)
	{
		setFrmCnt(oDetNd.getFrmCnt());
		setDetBBox(oDetNd.getDetBBox());
		setDetScr(oDetNd.getDetScr());
		setDetCls(oDetNd.getDetCls());
	}
	inline cv::Rect2f getBBox(void) { return m_oBBox; }
	inline void setBBox(cv::Rect2f oBBox) { m_oBBox = oBBox; }
	inline cv::Point2f get2dFtPt(void) { return m_o2dFtPt; }
	inline void set2dFtPt(cv::Point2f o2dFtPt) { m_o2dFtPt = o2dFtPt; }
	inline int getSt8(void) { return m_nSt8; }
	inline void setSt8(int nSt8) { m_nSt8 = nSt8; }
	inline bool getMtchTrkFlg(void) { return m_bMtchTrkFlg; }
	inline void setMtchTrkFlg(bool bMtchTrkFlg) { m_bMtchTrkFlg = bMtchTrkFlg; }
	inline std::vector<cv::Rect2f> getMtchBBoxs() { return m_voMtchBBox; }
	inline cv::Rect2f getMtchBBox(int nIdx) { return m_voMtchBBox[nIdx]; }
	inline void setMtchBBoxs(std::vector<cv::Rect2f> voBBox) { m_voMtchBBox = voBBox; }
	inline void addMtchBBox(cv::Rect2f oBBox) { m_voMtchBBox.push_back(oBBox); }
	inline int getMtchBBoxNum(void) { return m_voMtchBBox.size(); }

protected:
	//! object bounding box
	cv::Rect2f m_oBBox;
	//! 2D foot point
	cv::Point2f m_o2dFtPt;
	//! state of the object node
	int m_nSt8;
	//! flag of matching with tracking node
	bool m_bMtchTrkFlg;
	//! list of matched candidate bounding boxes
	std::vector<cv::Rect2f> m_voMtchBBox;
};

// tracking node
class CTrkNd : public CCandNd, public CTraj
{
public:
	CTrkNd(void);
	CTrkNd(int nFrmCnt, int nId, cv::Rect2f oBBox);
	CTrkNd(CCandNd oCandNd, int nSt8, int nId, int nNtrFrmCnt, cv::KalmanFilter oKF);
	~CTrkNd(void);

	inline void setCandNd(CCandNd oCandNd, int nSt8 = ND_ST8_DFLT)
	{
		setDetNd(oCandNd);
		setBBox(oCandNd.getBBox());
		set2dFtPt(oCandNd.get2dFtPt());
		setSt8(nSt8);
		setMtchTrkFlg(true);
	}
	inline int getId(void) { return m_nId; }
	inline void setId(int nId) { m_nId = nId; }
	inline int getNtrFrmCnt(void) { return m_nNtrFrmCnt; }
	inline void setNtrFrmCnt(int nNtrFrmCnt) { m_nNtrFrmCnt = nNtrFrmCnt; }
	inline bool getMtchCandFlg(void) { return m_bMtchCandFlg; }
	inline void setMtchCandFlg(bool bMtchCandFlg) { m_bMtchCandFlg = bMtchCandFlg; }
	inline bool getPredFlg(void) { return m_bPredFlg; }
	inline void setPredFlg(bool bPredFlg) { m_bPredFlg = bPredFlg; }
	inline double getVis(void) { return m_fVis; }
	inline void setVis(double fVis) { m_fVis = fVis; }
	inline int getDepIdx(void) { return m_nDepIdx; }
	inline void setDepIdx(int nDepthInd) { m_nDepIdx = nDepthInd; }
	inline cv::KalmanFilter getKF(void) { return m_oKF; }
	inline void setKF(cv::KalmanFilter oKF) { m_oKF = oKF; }
	inline void setKFTmDiff(double fTmDiff)
	{
		m_oKF.transitionMatrix.at<double>(0, 2) = fTmDiff;
		m_oKF.transitionMatrix.at<double>(1, 3) = fTmDiff;
	}
	inline void resetKFErrCovPre(void) { cv::setIdentity(m_oKF.errorCovPre, cv::Scalar::all(KF_ERR_COV_PRE)); }
	inline void setKFSt8Post(cv::Mat oSt8Vec) { m_oKF.statePost = oSt8Vec; }
	inline cv::Mat predKF(void) { return m_oKF.predict(); }
	inline void corrKF(cv::Mat oMeasVec) { m_oKF.correct(oMeasVec); }
	inline CTraj getTraj(void) { return CTraj(m_vnTrajFrmCnt, m_voTrajBBox, m_voTraj2dFtPt, m_vbTrajPredFlg); }
	inline void setTraj(CTraj oTraj)
	{
		setTrajFrmCnts(oTraj.getTrajFrmCnts());
		setTrajBBoxs(oTraj.getTrajBBoxs());
		setTraj2dFtPts(oTraj.getTraj2dFtPts());
		setTrajPredFlgs(oTraj.getTrajPredFlgs());
	}

protected:
	//! object identity
	int m_nId;
	//! the entering frame count
	int m_nNtrFrmCnt;
	//! flag of matching with a candidate node
	bool m_bMtchCandFlg;
	//! flag of tracking by prediction
	bool m_bPredFlg;
	//! the visibility (0 -> 1: more area visible)
	double m_fVis;
	//! the depth index (larger -> closer)
	int m_nDepIdx;
	//! Kalman filter
	cv::KalmanFilter m_oKF;
};

class CObjTrk
{
public:
	CObjTrk(void);
	~CObjTrk(void);

	//! initializes the tracker
	void initialize(CCfg oCfg);
	//! run tracking algorithm
	void process(std::vector<CDetNd>& voDetNd, int nFrmCnt);
	//! outputs tracking results
	void output(void);
	//! sets video frame rate
	inline void setFrmRt(double fFrmRt) { m_oCfg.setFrmRt(fFrmRt); }

private:
	//! detects enteringstd::ifstream tracking nodes
	void detNtrTrkNd(void);
	//! detects left nodes
	void detLftNd(void);
	//! sets the list of predicted nodes
	void initPredNdLs(void);
	//! initializes Kalman filter
	cv::KalmanFilter initKF(CCandNd oCandNd);
	//! performs Kalman filter tracking
	void trkKF(CTrkNd& oTrkNd);
	//! updates Kalman filter (matching each tracking node with candidate node)
	void updKF(CTrkNd& oTrkNd);
	//! performs tracking by prediction for unmatched tracking node(s)
	void trkPred(void);
	//! matches a tracking node with a candidate node
	void mtchTrkCand(CTrkNd& oTrkNd, cv::Rect2f oPredBBox, cv::Point2f oPred2dFtPt);
	//! initializes the list of candidate nodes
	void initCandNdLs(std::vector<CDetNd>& voDetNd);
	//! shifts the candidate node to a predicted 2D location
	bool shfCandNd(CCandNd oCandNd, CCandNd& oPredCandNd, cv::Point2f oPred2dFtPt, cv::Size2f oPred2dBBoxSz);
	//! generates depth map from the list of tracking nodes
	cv::Mat genDepMap(std::vector<CTrkNd> voTrkNd);
	//! generates depth map from the list of candidate nodes
	cv::Mat genDepMap(std::vector<CCandNd> voCandNd);
	//! calculates the visibility of tracking node
	double calcVis(cv::Rect2f oBBox, int iDep, cv::Mat oDepMap);
	//! outputs tracking results in text file in MOTChallenge format
	void outTxtMot(CTrkNd oTrkNd);
	//! outputs tracking results in text file in KITTI format
	void outTxtKitti(CTrkNd oTrkNd);
	//! sorts the tracking results in MOTChallenge format
	void sortTxtMot(void);
	//! reads object tracking results in MOTChallenge format
	bool rdObjTrkMot(std::vector<CTrkNd>& voTrkNd, int nFrmCnt);
	//! reads object tracking  results in KITTI format
	bool rdObjTrkKitti(std::vector<CTrkNd>& voTrkNd, int nFrmCnt);
	//! plots tracking bounding box(es) on the frame image
	void pltTrkBBox(cv::Mat& oImgFrm, int nFrmCnt, std::vector<CTrkNd> voTrkNd);

	//! configuration file
	CCfg m_oCfg;
	//! vector of colors of bounding boxes for plotting tracking results
	std::vector<cv::Scalar> m_voBBoxClr;
	//! current frame count
	int m_nFrmCnt;
	//! maximum object ID
	int m_nMaxId;
	//! ROI image
	cv::Mat m_oImgRoi;
	//! next tracking node
	CTrkNd m_oNxtTrkNd;
	//! path of output folder of tracking results
	char m_acOutTrkFlrPth[256];
	//! text file for writing tracking results
	std::ifstream m_ifsOutTrkTxt;
	//! list of candidate nodes in current frame
	std::vector<CCandNd> m_voCurrCandNd;
	//! list of candidate nodes in previous frame
	std::vector<CCandNd> m_voPrevCandNd;
	//! list of tracking nodes in current frame
	std::vector<CTrkNd> m_voCurrTrkNd;
	//! list of tracking nodes by prediction
	std::vector<CTrkNd> m_voPredNd;
};
