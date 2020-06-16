#include "detectFlow.h"

int LGPRegionDefectDetect(const Mat LGPRegion)
{
	Mat LGPRegionGray;
	cvtColor(LGPRegion, LGPRegionGray, CV_RGB2GRAY);

	// 微小瑕疵区域提取
	Mat gradX, gradY, regionGradX, regionGradY, LGPRegionEdge;
	Sobel(LGPRegionGray, gradX, CV_16S, 1, 0, 3);
	convertScaleAbs(gradX, regionGradX);
	Sobel(LGPRegionGray, gradY, CV_16S, 0, 1, 3);
	convertScaleAbs(gradY, regionGradY);
	addWeighted(regionGradX, 0.5, regionGradY, 0.5, 0, LGPRegionEdge);
	// 对边缘图像进行二值化，剔除一些微弱边缘
	Mat LGPEdgeBin;
	threshold(LGPRegionEdge, LGPEdgeBin, regionBinThresh, 255, CV_THRESH_BINARY);

	// 大瑕疵区域提取
	Mat LGPMeanBlur;  // 对图像进行均值滤波
	blur(LGPRegionGray, LGPMeanBlur, blurElementSize, Point(-1, -1));
	Mat LGPBlurDiff = abs(LGPMeanBlur - LGPRegionGray);
	Mat blurDiffBin;
	threshold(LGPBlurDiff, blurDiffBin, blurBinThresh, 255, CV_THRESH_BINARY);
	
	Mat blurDiffLabel = blurDiffBin / 255;
	vector<ROI> pointGroup;
	maskModify(blurDiffLabel, blurDiffBin, pointGroup);

	// 对导光板图像的四条边界进行瑕疵剔除
	borderDefectDetect(LGPRegionGray, pointGroup, blurDiffBin);
	
	// 将大小两种瑕疵的检测结果进行合并
	Mat LGPDefectBin = Mat::zeros(Size(LGPEdgeBin.cols, LGPEdgeBin.rows), CV_8UC1);
	LGPDefectBin = LGPEdgeBin + blurDiffBin;
	Mat LGPMaskLabel = blurDiffBin / 255;  // 定义二值掩模

	// 显示瑕疵区域
	Mat LGPDefectShow = LGPRegion.clone();
	LGPMaskLabel = 1 - LGPMaskLabel;
	// 分割图像通道并对R通道赋值
	vector<Mat> LGPChannels;
	split(LGPDefectShow, LGPChannels);
	for (int i = 0; i < 3; i++)
	{
		LGPChannels[i] = LGPChannels[i].mul(LGPMaskLabel);
	}
	LGPChannels[2] += blurDiffBin;  // 对R通道的瑕疵区域赋值为255

	merge(LGPChannels, LGPDefectShow);

	imshow("LGPDefectShow", LGPDefectShow);

	return processOK;
}

int maskModify(Mat& LGPMaskLabel, Mat& LGPOtsuBin, vector<ROI>& pointGroup)
{
	// 依次读取掩模中的像素，剔除小区域
	int height = LGPOtsuBin.rows;
	int width = LGPOtsuBin.cols;
	// 从左上角开始依次读取像素点
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			int label = (int)LGPMaskLabel.at<uchar>(i, j);
			Point coord = Point(j, i);
			// 根据区域内瑕疵点的个数确定需要保留的部分
			if (label == 1)
			{
				particleRemove(LGPMaskLabel, coord, LGPOtsuBin, pointGroup);
			}
		}
	}

	return processOK;
}

/*找出连通的瑕疵单元个数，如果大于三个则认为是瑕疵点*/
int particleRemove(Mat& LGPMaskLabel, const Point coord, Mat& LGPOtsuBin,
	vector<ROI>& pointGroup)
{
	int labelX, labelY;  // 当前访问的标签元素
	vector<Point> regionPtGroup;
	regionPtGroup.push_back(coord);
	int indexVis = 0;  // 当前访问元素的索引
	vector<Point>::iterator iterVis = regionPtGroup.begin();
	// 定义区域左上角和右下角的顶点坐标
	Point leftUpPoint = { LGPOtsuBin.cols,LGPOtsuBin.rows };
	Point rightDownPoint = { -1,-1 };
	// 当首尾迭代器相遇时停止工作
	while (iterVis != regionPtGroup.end())
	{
		labelX = (*iterVis).x;  //提取当前元素的坐标
		labelY = (*iterVis).y;
		LGPMaskLabel.at<uchar>(labelY, labelX) = 0;
		leftUpPoint.x = leftUpPoint.x < labelX ? leftUpPoint.x : labelX;
		leftUpPoint.y = leftUpPoint.y < labelY ? leftUpPoint.y : labelY;
		rightDownPoint.x = rightDownPoint.x > labelX ? rightDownPoint.x : labelX;
		rightDownPoint.y = rightDownPoint.y > labelY ? rightDownPoint.y : labelY;
		// 遍历周围的四个单元，如果有瑕疵单元则放入容器
		for (int i = -1;i <= 1;i++)
		{
			for (int j = -1;j <= 1;j++)
			{
				// 不访问当前位置的点
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
						if (labelAdj == 1)  // 还未访问过的瑕疵单元
						{
							Point posAdj = Point(adjacentX, adjacentY);
							regionPtGroup.push_back(posAdj);
							// 将访问过的瑕疵单元标签赋为2
							LGPMaskLabel.at<uchar>(adjacentY, adjacentX) = 2;
						}
					}
				}
			}
		}
		indexVis++;  //每循环一次都访问了一个元素
		iterVis = regionPtGroup.begin() + indexVis;  //更新迭代器的位置
	}

	// 将瑕疵单元所在的区域框出
	int regionWidth = rightDownPoint.x - leftUpPoint.x + 1;
	int regionHeight = rightDownPoint.y - leftUpPoint.y + 1;
	// 如果该连通域的边长均小于3，则将该区域的给消除掉
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

	// 对灰度图像进行3*3的均值滤波
	Mat LGPMeanFilter;
	blur(LGPRegionGray, LGPMeanFilter, Size(3, 3), Point(-1, -1), BORDER_DEFAULT);

	// 依次对每个轮廓进行判别处理
	for (int i = 0;i < pointGroup.size();i++)
	{
		// 判断是否为边界
		if (pointGroup[i].leftUpX<borderOffset || pointGroup[i].leftUpY<borderOffset ||
			pointGroup[i].rightDownX>width - borderOffset ||
			pointGroup[i].rightDownY>height - borderOffset)
		{
			// 对contours[i]中的所有点对应的灰度值求和
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