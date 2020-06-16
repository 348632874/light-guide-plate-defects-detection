#pragma once

#include<iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

#define processOK 1
#define error -1

// ����ROI�ṹ��
struct ROI
{
	vector<Point> regionPt;
	// ��������������Ϻ����µ�����
	int leftUpX;
	int leftUpY;
	int rightDownX;
	int rightDownY;
};

// �������ű���
const float resizeScale = 0.5;
const int binThresh = 30;
// ����Hough�任�Ĳ��ֲ���
const int linePointNum = 80;
const int minLineLength = 150;
const int minLineGap = 10;
const int borderOffset = 3;
const int minHighLightDiff = 30;

// ����sideLine
struct sideLine
{
	Vec4i line;  // ֱ�ߵĶ˵���Ϣ
	float centerDis;  // ֱ�����ĵ�ͼ�����ĵľ���
};

const int minSideLength = 3;
const int minRegionSize = 10;

// ����LGP������߿�����ڼ�⵽��ֱ�ߵ�ƫ����
static int lineOffset = 2;
const Size blurElementSize = Size(15, 15);
const int sideDetectDis = 10;
const int regionBinThresh = 20;
const int blurBinThresh = 3;
const int gaborBinThresh = 30;
const int freqGaussBin = 0.55;
const int borderBinThresh = 10;
const int minBorderOffset = 10;
