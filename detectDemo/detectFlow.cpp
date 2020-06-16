#include "detectFlow.h"

int LGPRegionDefectDetect(const Mat LGPRegion)
{
	Mat LGPRegionGray;
	cvtColor(LGPRegion, LGPRegionGray, CV_RGB2GRAY);

	// ΢С覴�������ȡ
	Mat gradX, gradY, regionGradX, regionGradY, LGPRegionEdge;
	Sobel(LGPRegionGray, gradX, CV_16S, 1, 0, 3);
	convertScaleAbs(gradX, regionGradX);
	Sobel(LGPRegionGray, gradY, CV_16S, 0, 1, 3);
	convertScaleAbs(gradY, regionGradY);
	addWeighted(regionGradX, 0.5, regionGradY, 0.5, 0, LGPRegionEdge);
	// �Ա�Եͼ����ж�ֵ�����޳�һЩ΢����Ե
	Mat LGPEdgeBin;
	threshold(LGPRegionEdge, LGPEdgeBin, regionBinThresh, 255, CV_THRESH_BINARY);

	// ��覴�������ȡ
	Mat LGPMeanBlur;  // ��ͼ����о�ֵ�˲�
	blur(LGPRegionGray, LGPMeanBlur, blurElementSize, Point(-1, -1));
	Mat LGPBlurDiff = abs(LGPMeanBlur - LGPRegionGray);
	Mat blurDiffBin;
	threshold(LGPBlurDiff, blurDiffBin, blurBinThresh, 255, CV_THRESH_BINARY);
	
	Mat blurDiffLabel = blurDiffBin / 255;
	vector<ROI> pointGroup;
	maskModify(blurDiffLabel, blurDiffBin, pointGroup);

	// �Ե����ͼ��������߽����覴��޳�
	borderDefectDetect(LGPRegionGray, pointGroup, blurDiffBin);
	
	// ����С����覴õļ�������кϲ�
	Mat LGPDefectBin = Mat::zeros(Size(LGPEdgeBin.cols, LGPEdgeBin.rows), CV_8UC1);
	LGPDefectBin = LGPEdgeBin + blurDiffBin;
	Mat LGPMaskLabel = blurDiffBin / 255;  // �����ֵ��ģ

	// ��ʾ覴�����
	Mat LGPDefectShow = LGPRegion.clone();
	LGPMaskLabel = 1 - LGPMaskLabel;
	// �ָ�ͼ��ͨ������Rͨ����ֵ
	vector<Mat> LGPChannels;
	split(LGPDefectShow, LGPChannels);
	for (int i = 0; i < 3; i++)
	{
		LGPChannels[i] = LGPChannels[i].mul(LGPMaskLabel);
	}
	LGPChannels[2] += blurDiffBin;  // ��Rͨ����覴�����ֵΪ255

	merge(LGPChannels, LGPDefectShow);

	imshow("LGPDefectShow", LGPDefectShow);

	return processOK;
}

int maskModify(Mat& LGPMaskLabel, Mat& LGPOtsuBin, vector<ROI>& pointGroup)
{
	// ���ζ�ȡ��ģ�е����أ��޳�С����
	int height = LGPOtsuBin.rows;
	int width = LGPOtsuBin.cols;
	// �����Ͻǿ�ʼ���ζ�ȡ���ص�
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			int label = (int)LGPMaskLabel.at<uchar>(i, j);
			Point coord = Point(j, i);
			// ����������覴õ�ĸ���ȷ����Ҫ�����Ĳ���
			if (label == 1)
			{
				particleRemove(LGPMaskLabel, coord, LGPOtsuBin, pointGroup);
			}
		}
	}

	return processOK;
}

/*�ҳ���ͨ��覴õ�Ԫ���������������������Ϊ��覴õ�*/
int particleRemove(Mat& LGPMaskLabel, const Point coord, Mat& LGPOtsuBin,
	vector<ROI>& pointGroup)
{
	int labelX, labelY;  // ��ǰ���ʵı�ǩԪ��
	vector<Point> regionPtGroup;
	regionPtGroup.push_back(coord);
	int indexVis = 0;  // ��ǰ����Ԫ�ص�����
	vector<Point>::iterator iterVis = regionPtGroup.begin();
	// �����������ϽǺ����½ǵĶ�������
	Point leftUpPoint = { LGPOtsuBin.cols,LGPOtsuBin.rows };
	Point rightDownPoint = { -1,-1 };
	// ����β����������ʱֹͣ����
	while (iterVis != regionPtGroup.end())
	{
		labelX = (*iterVis).x;  //��ȡ��ǰԪ�ص�����
		labelY = (*iterVis).y;
		LGPMaskLabel.at<uchar>(labelY, labelX) = 0;
		leftUpPoint.x = leftUpPoint.x < labelX ? leftUpPoint.x : labelX;
		leftUpPoint.y = leftUpPoint.y < labelY ? leftUpPoint.y : labelY;
		rightDownPoint.x = rightDownPoint.x > labelX ? rightDownPoint.x : labelX;
		rightDownPoint.y = rightDownPoint.y > labelY ? rightDownPoint.y : labelY;
		// ������Χ���ĸ���Ԫ�������覴õ�Ԫ���������
		for (int i = -1;i <= 1;i++)
		{
			for (int j = -1;j <= 1;j++)
			{
				// �����ʵ�ǰλ�õĵ�
				if (i != 0 || j != 0)
				{
					int adjacentX = labelX + i;
					int adjacentY = labelY + j;
					if (adjacentY >= 0 && adjacentY < LGPMaskLabel.rows
						&&adjacentX >= 0 && adjacentX < LGPMaskLabel.cols)
					{
						int labelAdj = (int)LGPMaskLabel.at<uchar>(adjacentY, adjacentX);
						/* cout << Point(adjacentX, adjacentY)
						<< LGPMaskLabel.at<int>(adjacentY, adjacentX) << " "; */
						if (labelAdj == 1)  // ��δ���ʹ���覴õ�Ԫ
						{
							Point posAdj = Point(adjacentX, adjacentY);
							regionPtGroup.push_back(posAdj);
							// �����ʹ���覴õ�Ԫ��ǩ��Ϊ2
							LGPMaskLabel.at<uchar>(adjacentY, adjacentX) = 2;
						}
					}
				}
			}
		}
		indexVis++;  //ÿѭ��һ�ζ�������һ��Ԫ��
		iterVis = regionPtGroup.begin() + indexVis;  //���µ�������λ��
	}

	// ��覴õ�Ԫ���ڵ�������
	int regionWidth = rightDownPoint.x - leftUpPoint.x + 1;
	int regionHeight = rightDownPoint.y - leftUpPoint.y + 1;
	// �������ͨ��ı߳���С��3���򽫸�����ĸ�������
	int regionSize = regionPtGroup.size();
	if ((regionWidth <= minSideLength || regionHeight <= minSideLength) || regionSize < minRegionSize)
	{
		vector<Point>::iterator iterPos = regionPtGroup.begin();
		while (iterPos != regionPtGroup.end())
		{
			int backPtX = (*iterPos).x;
			int backPtY = (*iterPos).y;
			iterPos++;
			LGPOtsuBin.at<uchar>(backPtY, backPtX) = 0;
		}
	}
	else
	{
		ROI region;
		region.regionPt = regionPtGroup;
		region.leftUpX = leftUpPoint.x;
		region.leftUpY = leftUpPoint.y;
		region.rightDownX = rightDownPoint.x;
		region.rightDownY = rightDownPoint.y;
		pointGroup.push_back(region);
	}

	return processOK;
}

int borderDefectDetect(const Mat LGPRegionGray, const vector<ROI> pointGroup, Mat& blurDiffBin)
{
	int width = LGPRegionGray.cols;
	int height = LGPRegionGray.rows;

	// �ԻҶ�ͼ�����3*3�ľ�ֵ�˲�
	Mat LGPMeanFilter;
	blur(LGPRegionGray, LGPMeanFilter, Size(3, 3), Point(-1, -1), BORDER_DEFAULT);

	// ���ζ�ÿ�����������б���
	for (int i = 0;i < pointGroup.size();i++)
	{
		// �ж��Ƿ�Ϊ�߽�
		if (pointGroup[i].leftUpX<borderOffset || pointGroup[i].leftUpY<borderOffset ||
			pointGroup[i].rightDownX>width - borderOffset ||
			pointGroup[i].rightDownY>height - borderOffset)
		{
			// ��contours[i]�е����е��Ӧ�ĻҶ�ֵ���
			/*int grayMean = 0;
			for (int n = 0; n < pointGroup[i].regionPt.size(); n++)
			{
				Point pt = pointGroup[i].regionPt[n];
				grayMean += (int)LGPRegionGray.at<uchar>(pt);
			}
			grayMean = grayMean / pointGroup[i].regionPt.size();*/
			for (int n = 0; n < pointGroup[i].regionPt.size(); n++)
			{
				Point pt = pointGroup[i].regionPt[n];
				int grayDiff = (int)LGPRegionGray.at<uchar>(pt) -
					(int)LGPMeanFilter.at<uchar>(pt);
				if (grayDiff <= minHighLightDiff)
				{
					blurDiffBin.at<uchar>(pt) = 0;
				}
				else
				{
					blurDiffBin.at<uchar>(pt) = 255;
				}
			}
		}
	}

	return processOK;
}