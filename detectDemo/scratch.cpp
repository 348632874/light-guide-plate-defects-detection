# include "scratch.h"
#include "detectFlow.h"

int scratchDetect(const Mat LGPRegion)
{
	Mat LGPRegionGray;
	cvtColor(LGPRegion, LGPRegionGray, CV_RGB2GRAY);

	// �ԻҶ�ͼ�����GammaУ��
	float fGamma = 0.5;
	Mat LGPGammaCor;
	gammaCorrection(LGPRegionGray, LGPGammaCor, fGamma);

	// ͼ����ֵ�˲�
	Mat LGPMedBlur;
	medianBlur(LGPGammaCor, LGPMedBlur, 3);

	// ��ͼ�����˫���˲����˳�������ͬʱ������Ե
	Mat LGPBilater;
	bilateralFilter(LGPMedBlur, LGPBilater, -1, 3, 3, BORDER_DEFAULT);

	// ��覴�������ȡ
	Mat LGPMeanBlur;  // ��ͼ����о�ֵ�˲�
	blur(LGPBilater, LGPMeanBlur, Size(15, 15), Point(-1, -1));
	Mat LGPBlurDiff = abs(LGPMeanBlur - LGPBilater);
	Mat blurDiffBin;
	threshold(LGPBlurDiff, blurDiffBin, blurBinThresh, 255, CV_THRESH_BINARY);

	vector<Mat> gaborFilterRes;
	// �ֱ��Բ�ͬ�Ƕȵ�gabor�˲����Ӷ�ͼ������˲�
	for (int k = 0; k < 4; k++)
	{
		Mat gaborKernel = getGaborKernel(Size(3, 3), 5, 0.25*double(k), 5, 0.5, CV_PI*0.5, CV_64F);
		Mat filterRes;
		filter2D(LGPGammaCor, filterRes, -1, gaborKernel, Point(-1, -1), 0, BORDER_DEFAULT);
		gaborFilterRes.push_back(filterRes);
	}

	// ���˲�������ֵ���б���
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

	// ��Gabor�˲����ͼ����ж�ֵ��
	Mat LGPGaborBin;
	threshold(LGPGabor, LGPGaborBin, gaborBinThresh, 255, CV_THRESH_BINARY);

	return processOK;
}

// ��ͼ�����GammaУ��
int gammaCorrection(const Mat srcImage, Mat& correctImage, float fGamma)
{
	// �������ұ�
	unsigned char lut[256];
	for (int i = 0; i < 256; i++)
	{
		lut[i] = static_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);
	}

	correctImage = srcImage.clone();
	// ��������������ͼ�񣬶�ÿ�������θ�ֵ
	MatIterator_<uchar> iter = correctImage.begin<uchar>();
	for (; iter != correctImage.end<uchar>(); iter++)
	{
		*iter = lut[(*iter)];
	}

	return processOK;
}