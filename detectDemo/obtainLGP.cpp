#include "obtainLGP.h"

int obtainLGP(const Mat LGPImage, Mat& LGPRegion)
{
	int LGPHeight = LGPImage.rows;
	int LGPWidth = LGPImage.cols;
	Mat LGPGray;
	cvtColor(LGPImage, LGPGray, CV_RGB2GRAY);

	// 利用Sobel算子求得边缘图像
	Mat gradX, absGradX, gradY, absGradY, LGPSobelEdge;
	Sobel(LGPGray, gradX, CV_16S, 1, 0, 3);
	convertScaleAbs(gradX, absGradX);
	Sobel(LGPGray, gradY, CV_16S, 0, 1, 3);
	convertScaleAbs(gradY, absGradY);
	addWeighted(absGradX, 0.5, absGradY, 0.5, 0, LGPSobelEdge);
	// 对边缘图像进行二值化，剔除一些微弱边缘
	Mat LGPOtsuBin;
	threshold(LGPSobelEdge, LGPOtsuBin, binThresh, 255, CV_THRESH_BINARY);

	// Hough变换边缘检测
	vector<Vec4i> LGPEdgeLines;
	HoughLinesP(LGPOtsuBin, LGPEdgeLines, 1, CV_PI / 180, linePointNum, minLineLength, minLineGap);
	// 保留最有可能的四条边
	Mat lineShowImage = LGPImage.clone();
	Vec4i LGPSide[4];  // 存储四条边
	vector<sideLine> sideClass[4];  // 根据边的角度和中心点位置将其分为四类（上下左右）
	Point LGPCenterPoint = Point(LGPWidth / 2, LGPHeight / 2);
	for (int i = 0; i < LGPEdgeLines.size(); i++)
	{
		Vec4i EdgeLine = LGPEdgeLines[i];
		line(lineShowImage, Point(EdgeLine[0], EdgeLine[1]),
			Point(EdgeLine[2], EdgeLine[3]), Scalar(255, 0, 0), 1, CV_AA);
		float angle;  // 直线的倾斜角
		if ((EdgeLine[0] - EdgeLine[2]) == 0)
		{
			angle = 180.0 / 2.0;
		}
		else
		{
			angle = atan(abs(1.0*(EdgeLine[1] - EdgeLine[3]) / (EdgeLine[0] - EdgeLine[2])))*180.0 / CV_PI;
		}
		Point midPoint = Point((EdgeLine[0] + EdgeLine[2]) / 2, (EdgeLine[1] + EdgeLine[3]) / 2);
		float centerDis = sqrt(pow(LGPCenterPoint.x - midPoint.x, 2) + pow(LGPCenterPoint.y - midPoint.y, 2));
		sideLine lineInfo = { EdgeLine,centerDis };
		if (midPoint.y <= LGPCenterPoint.y)
		{
			if (angle <= 45)
			{
				sideClass[0].push_back(lineInfo);
			}
			else if (angle > 45)
			{
				if (midPoint.x <= LGPCenterPoint.x)
				{
					sideClass[2].push_back(lineInfo);
				}
				else if (midPoint.x > LGPCenterPoint.x)
				{
					sideClass[3].push_back(lineInfo);
				}
			}
		}
		else if (midPoint.y > LGPCenterPoint.y)
		{
			if (angle <= 45)
			{
				sideClass[1].push_back(lineInfo);
			}
			else if (angle > 45)
			{
				if (midPoint.x <= LGPCenterPoint.x)
				{
					sideClass[2].push_back(lineInfo);
				}
				else if (midPoint.x > LGPCenterPoint.x)
				{
					sideClass[3].push_back(lineInfo);
				}
			}
		}
		else
		{
			continue;
		}
	}

	// 对四类边所在的容器依次进行排序
	for (int i = 0; i < 4; i++)
	{
		vector<sideLine> line = sideClass[i];
		if (line.size() == 0)
		{
			return error;
		}
		sort(line.begin(), line.end(), sortFun);

		sideClass[i] = line;
	}
	// 将每一类的第一条边（最接近中心的边）作为LGP的四条边
	for (int i = 0; i < 4; i++)
	{
		lineOffset = pow(-1, i)*lineOffset;
		LGPSide[i] = { sideClass[i][0].line[0] - lineOffset, sideClass[i][0].line[1] - lineOffset,
			sideClass[i][0].line[2] - lineOffset ,sideClass[i][0].line[3] - lineOffset };

		line(lineShowImage, Point(LGPSide[i][0], LGPSide[i][1]),
			Point(LGPSide[i][2], LGPSide[i][3]), Scalar(0, 0, 255), 2, CV_AA);
	}

	// 通过四条边定位整个导光板所在的区域
	locateLGPRegion(LGPSide, LGPImage, LGPRegion);

	return 0;
}

// 自定义排序函数  
bool sortFun(sideLine line1, sideLine line2)
{
	return line1.centerDis < line2.centerDis;
}

// 定位导光板所在的区域
int locateLGPRegion(const Vec4i* LGPSide, const Mat LGPImage, Mat& LGPRegion)
{
	// 求出上下两条直线的相关信息（中心点与斜率）
	Point upLineCenter = { (LGPSide[0][0] + LGPSide[0][2]) / 2,
		(LGPSide[0][1] + LGPSide[0][3]) / 2 };
	Point downLineCenter = { (LGPSide[1][0] + LGPSide[1][2]) / 2,
		(LGPSide[1][1] + LGPSide[1][3]) / 2 };
	float upRatio = 1.0*(LGPSide[0][1] - LGPSide[0][3]) / (LGPSide[0][0] - LGPSide[0][2]);
	float downRatio = 1.0*(LGPSide[1][1] - LGPSide[1][3]) / (LGPSide[1][0] - LGPSide[1][2]);

	// 将中心点按斜率向左右两边延伸，找出与两条边的交点
	float minPointLineDis = LGPImage.cols;  // 点到直线的最小距离
	Point2f LGPCorner[4];  // LGP的四个顶角坐标
	for (int i = 0; i <= upLineCenter.x; i++)
	{
		float curPointX = upLineCenter.x - i;
		float curPointY = upLineCenter.y - i*upRatio;
		float pointLineDis;
		calcPointLineDis(LGPSide[2], { curPointX, curPointY }, pointLineDis);

		if (pointLineDis <= minPointLineDis)
		{
			minPointLineDis = pointLineDis;
		}
		else if (pointLineDis > minPointLineDis&&pointLineDis < lineOffset)
		{
			LGPCorner[0] = { curPointX, curPointY };
			break;
		}
	}
	minPointLineDis = LGPImage.cols;
	for (int i = 0; i < LGPImage.cols - upLineCenter.x; i++)
	{
		float curPointX = upLineCenter.x + i;
		float curPointY = upLineCenter.y + i*upRatio;
		float pointLineDis;
		calcPointLineDis(LGPSide[3], { curPointX, curPointY }, pointLineDis);

		if (pointLineDis <= minPointLineDis)
		{
			minPointLineDis = pointLineDis;
		}
		else if (pointLineDis > minPointLineDis&&pointLineDis < lineOffset)
		{
			LGPCorner[1] = { curPointX, curPointY };
			break;
		}
	}
	minPointLineDis = LGPImage.cols;
	for (int i = 0; i <= downLineCenter.x; i++)
	{
		float curPointX = downLineCenter.x - i;
		float curPointY = downLineCenter.y - i*downRatio;
		float pointLineDis;
		calcPointLineDis(LGPSide[2], { curPointX, curPointY }, pointLineDis);

		if (pointLineDis <= minPointLineDis)
		{
			minPointLineDis = pointLineDis;
		}
		else if (pointLineDis > minPointLineDis&&pointLineDis < lineOffset)
		{
			LGPCorner[2] = { curPointX, curPointY };
			break;
		}
	}
	minPointLineDis = LGPImage.cols;
	for (int i = 0; i < LGPImage.cols - downLineCenter.x; i++)
	{
		float curPointX = downLineCenter.x + i;
		float curPointY = downLineCenter.y + i*downRatio;
		float pointLineDis;
		calcPointLineDis(LGPSide[3], { curPointX, curPointY }, pointLineDis);

		if (pointLineDis <= minPointLineDis)
		{
			minPointLineDis = pointLineDis;
		}
		else if (pointLineDis > minPointLineDis&&pointLineDis < lineOffset)
		{
			LGPCorner[3] = { curPointX, curPointY };
			break;
		}
	}

	// 选取宽和高作为区域提取的边长，这一步会导致局部点的变化
	int regionWidth = sqrt(pow((LGPCorner[1] - LGPCorner[0]).x, 2) +
		pow((LGPCorner[1] - LGPCorner[0]).y, 2));
	int regionHeight = sqrt(pow((LGPCorner[2] - LGPCorner[0]).x, 2) +
		pow((LGPCorner[2] - LGPCorner[0]).y, 2));
	LGPRegion = Mat::zeros(Size(regionWidth, regionHeight), CV_8UC3);
	Point2f regionCorner[4] = { { 0.0, 0.0 },{ float(regionWidth - 1), 0.0 },
	{ 0.0, float(regionHeight - 1) },{ float(regionWidth - 1), float(regionHeight - 1) } };
	// 透视变换提取LGP区域
	Mat persTransMat = getPerspectiveTransform(LGPCorner, regionCorner);
	warpPerspective(LGPImage, LGPRegion, persTransMat, Size(LGPRegion.cols, LGPRegion.rows), CV_INTER_CUBIC);

	return processOK;
}


// 求出点到直线的距离
int calcPointLineDis(Vec4i line, Point2f point, float& pointLineDis)
{
	// 求直线方程的系数
	int coefA = 0, coefB = 0, constantC = 0;
	coefA = line[1] - line[3];
	coefB = line[2] - line[0];
	constantC = line[0] * line[3] - line[1] * line[2];
	// 计算点到直线的距离
	pointLineDis = ((float)abs(coefA*point.x + coefB*point.y + constantC))
		/ ((float)sqrtf(coefA*coefA + coefB*coefB));

	return processOK;
}

