#include "obtainLGP.h"
#include "scratch.h"
#include "detectFlow.h"

int main()
{
	int imageID = 2;
	Mat LGPImage = imread("LGP image//LGP0925//dirty//" + to_string(imageID) + ".bmp");

	// 通过四条边定位整个导光板所在的区域
	Mat LGPRegion;
	obtainLGP(LGPImage, LGPRegion);
	imwrite("LGPRegion.bmp", LGPRegion);

	// 对图像中的划伤进行瑕疵检测
	scratchDetect(LGPRegion);

	// 对图像中的瑕疵区域进行检测
	LGPRegionDefectDetect(LGPRegion);

	waitKey(0);
}