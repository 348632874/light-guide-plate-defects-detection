# include "scratch.h"
#include "detectFlow.h"

int scratchDetect(const Mat LGPRegion)
{
	Mat LGPRegionGray;
	cvtColor(LGPRegion, LGPRegionGray, CV_RGB2GRAY);

	// 对灰度图像进行Gamma校正
	float fGamma = 0.5;
	Mat LGPGammaCor;
	gammaCorrection(LGPRegionGray, LGPGammaCor, fGamma);

	// 图像中值滤波
	Mat LGPMedBlur;
	medianBlur(LGPGammaCor, LGPMedBlur, 3);

	// 对图像进行双边滤波，滤除噪声的同时保留边缘
	Mat LGPBilater;
	bilateralFilter(LGPMedBlur, LGPBilater, -1, 3, 3, BORDER_DEFAULT);

	// 大瑕疵区域提取
	Mat LGPMeanBlur;  // 对图像进行均值滤波
	blur(LGPBilater, LGPMeanBlur, Size(15, 15), Point(-1, -1));
	Mat LGPBlurDiff = abs(LGPMeanBlur - LGPBilater);
	Mat blurDiffBin;
	threshold(LGPBlurDiff, blurDiffBin, blurBinThresh, 255, CV_THRESH_BINARY);

	vector<Mat> gaborFilterRes;
	// 分别以不同角度的gabor滤波算子对图像进行滤波
	for (int k = 0; k < 4; k++)
	{
		Mat gaborKernel = getGaborKernel(Size(3, 3), 5, 0.25*double(k), 5, 0.5, CV_PI*0.5, CV_64F);
		Mat filterRes;
		filter2D(LGPGammaCor, filterRes, -1, gaborKernel, Point(-1, -1), 0, BORDER_DEFAULT);
		gaborFilterRes.push_back(filterRes);
	}

	// 将滤波后的最大值进行保留
	Mat LGPGabor = Mat::zeros(Size(LGPGammaCor.cols, LGPGammaCor.rows), CV_8UC1);
	for (int i = 0; i < LGPGammaCor.rows; i++)
	{
		for (int j = 0; j < LGPGammaCor.cols; j++)
		{
			for (int n = 0; n < 3; n++)
			{
				LGPGabor.at<uchar>(i, j) = max((int)gaborFilterRes[n].at<uchar>(i, j),
					(int)LGPGabor.at<uchar>(i, j));
			}
		}
	}

	// 对Gabor滤波后的图像进行二值化
	Mat LGPGaborBin;
	threshold(LGPGabor, LGPGaborBin, gaborBinThresh, 255, CV_THRESH_BINARY);

	return processOK;
}

// 对图像进行Gamma校正
int gammaCorrection(const Mat srcImage, Mat& correctImage, float fGamma)
{
	// 构建查找表
	unsigned char lut[256];
	for (int i = 0; i < 256; i++)
	{
		lut[i] = static_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);
	}

	correctImage = srcImage.clone();
	// 构建迭代器遍历图像，对每个点依次赋值
	MatIterator_<uchar> iter = correctImage.begin<uchar>();
	for (; iter != correctImage.end<uchar>(); iter++)
	{
		*iter = lut[(*iter)];
	}

	return processOK;
}