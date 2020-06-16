# light-guide-plate-defects-detection
A project for light guide plate's defects detection.

# instruction
Opencv should be installed for image processing.<br>
Light guide plate dataset can be obtained in <https://pan.baidu.com/s/1ZT0HBDNUw77jptAUdg-CqA>, and the password is 7yt6.<br>
The dataset should be placed in the path "detectDemo/LGP image".
Run main.cpp to get detection results of light guide plate defects.

# introduction
Our project mainly contains two parts, the first part is to locate light guide plate from the image we acquire, and the second part is to detect defects in the light guide plate. Sobel algorithm is used to generate edge image, and Hough transform is used to detect plate border. Finally, we get the region of guide light plate in the image. When it comes to defects detection, we use Sobel, binaryzation, mean filter and some other algorithm to locate defects. The detection results can be shown as follows:<br>
