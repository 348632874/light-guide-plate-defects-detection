#pragma once

# include "define.h"
#include <algorithm>

int scratchDetect(const Mat LGPRegion);

int gammaCorrection(const Mat srcImage, Mat& correctImage, float fGamma);

// 对图像转换到频域后滤波，再逆变换
int frequencyFilter(const Mat inputImage, Mat& LGPFreqFilter);