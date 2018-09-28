#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Cfg.h"

//! the radius of each node drawn on the image
#define ND_RAD	6

class CRoiSel
{
public:
	//! full constructor
	CRoiSel() { m_bPlgnCmpl = false; m_oImgCp = cv::Mat(); m_oImgExp = cv::Mat(); m_oImgRoi = cv::Mat();}
	//! default destructor
	~CRoiSel() {}
	
	//! pushes a node to node-list and draw the line & circle
	void addNd(int nX, int nY);	
	//! select ROI image
	bool selRoi(const cv::Mat& oImgOrig, CCfg& oCfg);

	inline bool chkImgLd() {if (!m_oImgExp.empty()) return true; else return false; }
	inline bool getPlgnCmpl() { return m_bPlgnCmpl; }

private:
	//! generate the ROI image
	void genRoi(char* acROIName); // called by selRoi()

	//! list of nodes
	std::vector<cv::Point> m_voNd;
	//! check if polygon is completed
	bool m_bPlgnCmpl;
	//! original image for drawing the polygon
	cv::Mat m_oImgCp;
	//! expanded image for drawing the polygon
	cv::Mat m_oImgExp;
	//! image for out
	cv::Mat m_oImgRoi;
};

extern CRoiSel oRoiSel;