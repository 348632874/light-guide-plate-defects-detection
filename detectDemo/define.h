#pragma once

#include<iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

#define processOK 1
#define error -1

// 定义ROI结构体
struct ROI
{
	vector<Point> regionPt;
	// 定义外接区域左上和右下的坐标
	int leftUpX;
	int leftUpY;
	int rightDownX;
	int rightDownY;
};

// 定义缩放比例
const float resizeScale = 0.5;
const int binThresh = 30;
// 定义Hough变换的部分参数
const int linePointNum = 80;
const int minLineLength = 150;
const int minLineGap = 10;
const int borderOffset = 3;
const int minHighLightDiff = 30;

// 定义sideLine
struct sideLine
{
	Vec4i line;  // 直线的端点信息
	float centerDis;  // 直线中心到图像中心的距离
};

const int minSideLength = 3;
const int minRegionSize = 10;

// 定义LGP四条外边框相对于检测到的直线的偏移量
static int lineOffset = 2;
const Size blurElementSize = Size(15, 15);
const int sideDetectDis = 10;
const int regionBinThresh = 20;
const int blurBinThresh = 3;
const int gaborBinThresh = 30;
const int freqGaussBin = 0.55;
const int borderBinThresh = 10;
const int minBorderOffset = 10;
