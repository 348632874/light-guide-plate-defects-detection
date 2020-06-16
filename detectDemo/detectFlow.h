#pragma once

#include "define.h"

int LGPRegionDefectDetect(const Mat LGPRegion);

int maskModify(Mat& LGPMaskLabel, Mat& LGPOtsuBin, vector<ROI>& pointGroup);

int particleRemove(Mat& LGPMaskLabel, const Point coord, Mat& LGPOtsuBin,
	vector<ROI>& pointGroup);

int borderDefectDetect(const Mat LGPRegionGray, const vector<ROI> pointGroup, Mat& blurDiffBin);