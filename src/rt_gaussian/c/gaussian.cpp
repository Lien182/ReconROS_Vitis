extern "C" {
#include "reconos_thread.h"
#include "reconos_calls.h"
}

#include "opencv2/opencv.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;
using namespace std;

extern "C" THREAD_ENTRY(); // this is required because of the mixture of c and c++


static void PublishMat(cv::Mat &img, sensor_msgs__msg__Image *pub_msg)
{
    for(int i=0; i < img.rows; i++)
        memcpy(pub_msg->data.data + i*img.cols*3, img.ptr(i,0), img.cols*3);

    pub_msg->width = img.cols;
    pub_msg->height = img.rows;
    pub_msg->step  = img.cols*3;
    pub_msg->data.size = img.cols*3*img.rows;
}

THREAD_ENTRY() {
	
		while(1)
		{
			ROS_SUBSCRIBE_TAKE(rgaussianblur_subdata, rgaussianblur_camera_image);
            cv::Mat cv_image_original = cv::Mat(rgaussianblur_camera_image->height, rgaussianblur_camera_image->width, CV_8UC3, rgaussianblur_camera_image->data.data);
            printf("Got msg \n");
            cv::GaussianBlur(cv_image_original, cv_image_original, Size(7, 7), 0.8666f);
             printf("Msg filtered \n");
            PublishMat(cv_image_original, rgaussianblur_camera_image);
            printf("Now publish! \n");
            ROS_PUBLISH(rgaussianblur_pubout, rgaussianblur_camera_image);
            
		}



	return;
}
