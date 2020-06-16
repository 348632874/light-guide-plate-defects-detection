#pragma once

#include "define.h"

int obtainLGP(const Mat LGPImage, Mat& LGPRegion);

bool sortFun(sideLine line1, sideLine line2);

int locateLGPRegion(const Vec4i* LGPSide, const Mat LGPImage, Mat& LGPRegion);

// 求出点到直线的距离
int calcPointLineDis(Vec4i line, Point2f point, float& pointLineDis);