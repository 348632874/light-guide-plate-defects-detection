#pragma once

# include "define.h"
#include <algorithm>

int scratchDetect(const Mat LGPRegion);

int gammaCorrection(const Mat srcImage, Mat& correctImage, float fGamma);

// ��ͼ��ת����Ƶ����˲�������任
int frequencyFilter(const Mat inputImage, Mat& LGPFreqFilter);