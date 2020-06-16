#include "obtainLGP.h"
#include "scratch.h"
#include "detectFlow.h"

int main()
{
	int imageID = 2;
	Mat LGPImage = imread("LGP image//LGP0925//dirty//" + to_string(imageID) + ".bmp");

	// ͨ�������߶�λ������������ڵ�����
	Mat LGPRegion;
	obtainLGP(LGPImage, LGPRegion);
	imwrite("LGPRegion.bmp", LGPRegion);

	// ��ͼ���еĻ��˽���覴ü��
	scratchDetect(LGPRegion);

	// ��ͼ���е�覴�������м��
	LGPRegionDefectDetect(LGPRegion);

	waitKey(0);
}