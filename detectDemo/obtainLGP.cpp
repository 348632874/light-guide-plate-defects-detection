#include "obtainLGP.h"

int obtainLGP(const Mat LGPImage, Mat& LGPRegion)
{
	int LGPHeight = LGPImage.rows;
	int LGPWidth = LGPImage.cols;
	Mat LGPGray;
	cvtColor(LGPImage, LGPGray, CV_RGB2GRAY);

	// ����Sobel������ñ�Եͼ��
	Mat gradX, absGradX, gradY, absGradY, LGPSobelEdge;
	Sobel(LGPGray, gradX, CV_16S, 1, 0, 3);
	convertScaleAbs(gradX, absGradX);
	Sobel(LGPGray, gradY, CV_16S, 0, 1, 3);
	convertScaleAbs(gradY, absGradY);
	addWeighted(absGradX, 0.5, absGradY, 0.5, 0, LGPSobelEdge);
	// �Ա�Եͼ����ж�ֵ�����޳�һЩ΢����Ե
	Mat LGPOtsuBin;
	threshold(LGPSobelEdge, LGPOtsuBin, binThresh, 255, CV_THRESH_BINARY);

	// Hough�任��Ե���
	vector<Vec4i> LGPEdgeLines;
	HoughLinesP(LGPOtsuBin, LGPEdgeLines, 1, CV_PI / 180, linePointNum, minLineLength, minLineGap);
	// �������п��ܵ�������
	Mat lineShowImage = LGPImage.clone();
	Vec4i LGPSide[4];  // �洢������
	vector<sideLine> sideClass[4];  // ���ݱߵĽǶȺ����ĵ�λ�ý����Ϊ���ࣨ�������ң�
	Point LGPCenterPoint = Point(LGPWidth / 2, LGPHeight / 2);
	for (int i = 0; i < LGPEdgeLines.size(); i++)
	{
		Vec4i EdgeLine = LGPEdgeLines[i];
		line(lineShowImage, Point(EdgeLine[0], EdgeLine[1]),
			Point(EdgeLine[2], EdgeLine[3]), Scalar(255, 0, 0), 1, CV_AA);
		float angle;  // ֱ�ߵ���б��
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

	// ����������ڵ��������ν�������
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
	// ��ÿһ��ĵ�һ���ߣ���ӽ����ĵıߣ���ΪLGP��������
	for (int i = 0; i < 4; i++)
	{
		lineOffset = pow(-1, i)*lineOffset;
		LGPSide[i] = { sideClass[i][0].line[0] - lineOffset, sideClass[i][0].line[1] - lineOffset,
			sideClass[i][0].line[2] - lineOffset ,sideClass[i][0].line[3] - lineOffset };

		line(lineShowImage, Point(LGPSide[i][0], LGPSide[i][1]),
			Point(LGPSide[i][2], LGPSide[i][3]), Scalar(0, 0, 255), 2, CV_AA);
	}

	// ͨ�������߶�λ������������ڵ�����
	locateLGPRegion(LGPSide, LGPImage, LGPRegion);

	return 0;
}

// �Զ���������  
bool sortFun(sideLine line1, sideLine line2)
{
	return line1.centerDis < line2.centerDis;
}

// ��λ��������ڵ�����
int locateLGPRegion(const Vec4i* LGPSide, const Mat LGPImage, Mat& LGPRegion)
{
	// �����������ֱ�ߵ������Ϣ�����ĵ���б�ʣ�
	Point upLineCenter = { (LGPSide[0][0] + LGPSide[0][2]) / 2,
		(LGPSide[0][1] + LGPSide[0][3]) / 2 };
	Point downLineCenter = { (LGPSide[1][0] + LGPSide[1][2]) / 2,
		(LGPSide[1][1] + LGPSide[1][3]) / 2 };
	float upRatio = 1.0*(LGPSide[0][1] - LGPSide[0][3]) / (LGPSide[0][0] - LGPSide[0][2]);
	float downRatio = 1.0*(LGPSide[1][1] - LGPSide[1][3]) / (LGPSide[1][0] - LGPSide[1][2]);

	// �����ĵ㰴б���������������죬�ҳ��������ߵĽ���
	float minPointLineDis = LGPImage.cols;  // �㵽ֱ�ߵ���С����
	Point2f LGPCorner[4];  // LGP���ĸ���������
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

	// ѡȡ��͸���Ϊ������ȡ�ı߳�����һ���ᵼ�¾ֲ���ı仯
	int regionWidth = sqrt(pow((LGPCorner[1] - LGPCorner[0]).x, 2) +
		pow((LGPCorner[1] - LGPCorner[0]).y, 2));
	int regionHeight = sqrt(pow((LGPCorner[2] - LGPCorner[0]).x, 2) +
		pow((LGPCorner[2] - LGPCorner[0]).y, 2));
	LGPRegion = Mat::zeros(Size(regionWidth, regionHeight), CV_8UC3);
	Point2f regionCorner[4] = { { 0.0, 0.0 },{ float(regionWidth - 1), 0.0 },
	{ 0.0, float(regionHeight - 1) },{ float(regionWidth - 1), float(regionHeight - 1) } };
	// ͸�ӱ任��ȡLGP����
	Mat persTransMat = getPerspectiveTransform(LGPCorner, regionCorner);
	warpPerspective(LGPImage, LGPRegion, persTransMat, Size(LGPRegion.cols, LGPRegion.rows), CV_INTER_CUBIC);

	return processOK;
}


// ����㵽ֱ�ߵľ���
int calcPointLineDis(Vec4i line, Point2f point, float& pointLineDis)
{
	// ��ֱ�߷��̵�ϵ��
	int coefA = 0, coefB = 0, constantC = 0;
	coefA = line[1] - line[3];
	coefB = line[2] - line[0];
	constantC = line[0] * line[3] - line[1] * line[2];
	// ����㵽ֱ�ߵľ���
	pointLineDis = ((float)abs(coefA*point.x + coefB*point.y + constantC))
		/ ((float)sqrtf(coefA*coefA + coefB*coefB));

	return processOK;
}

