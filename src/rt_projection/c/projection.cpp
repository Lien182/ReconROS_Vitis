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
	
        // if self.sub_image_type == "compressed":
        //     # converts compressed image to opencv image
        //     np_image_original = np.fromstring(msg_img.data, np.uint8)
        //     cv_image_original = cv2.imdecode(np_image_original, cv2.IMREAD_COLOR)
        // elif self.sub_image_type == "raw":
        //     # converts raw image to opencv image
        //     cv_image_original = self.cvBridge.imgmsg_to_cv2(msg_img, "bgr8")

        // # setting homography variables
        // top_x = self.top_x
        // top_y = self.top_y
        // bottom_x = self.bottom_x
        // bottom_y = self.bottom_y

        // if self.is_calibration_mode == True:
        //     
        // # adding Gaussian blur to the image of original
        // cv_image_original = cv2.GaussianBlur(cv_image_original, (5, 5), 0)

        // ## homography transform process
        // # selecting 4 points from the original image
        // pts_src = np.array([[160 - top_x, 180 - top_y], [160 + top_x, 180 - top_y], [160 + bottom_x, 120 + bottom_y], [160 - bottom_x, 120 + bottom_y]])

        // # selecting 4 points from image that will be transformed
        // pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])

        // # finding homography matrix
        // h, status = cv2.findHomography(pts_src, pts_dst)

        // # homography process
        // cv_image_homography = cv2.warpPerspective(cv_image_original, h, (1000, 600))

        // # fill the empty space with black triangles on left and right side of bottom
        // triangle1 = np.array([[0, 599], [0, 340], [200, 599]], np.int32)
        // triangle2 = np.array([[999, 599], [999, 340], [799, 599]], np.int32)
        // black = (0, 0, 0)
        // white = (255, 255, 255)
        // cv_image_homography = cv2.fillPoly(cv_image_homography, [triangle1, triangle2], black)

        // if self.pub_image_type == "compressed":
        //     # publishes ground-project image in compressed type
        //     self.pub_image_projected.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_homography, "jpg"))

        // elif self.pub_image_type == "raw":
        //     # publishes ground-project image in raw type
        //     self.pub_image_projected.publish(self.cvBridge.cv2_to_imgmsg(cv_image_homography, "bgr8"))

        // self.top_x = rospy.get_param("/camera/extrinsic_camera_calibration/top_x", 75)
        // self.top_y = rospy.get_param("/camera/extrinsic_camera_calibration/top_y", 35)
        // self.bottom_x = rospy.get_param("/camera/extrinsic_camera_calibration/bottom_x", 165)
        // self.bottom_y = rospy.get_param("/camera/extrinsic_camera_calibration/bottom_y", 120)


        int32_t top_x = 75*2;
        int32_t top_y = 35*2;
        int32_t bottom_x = 165*2;
        int32_t bottom_y = 120*2;

        uint8_t bIsCalibrated = 0;

		while(1)
		{
			ROS_SUBSCRIBE_TAKE(rcameraimageprojection_subdata, rcameraimageprojection_camera_image);
            cv::Mat cv_image_original = cv::Mat(rcameraimageprojection_camera_image->height, rcameraimageprojection_camera_image->width, CV_8UC3, rcameraimageprojection_camera_image->data.data);


            if(bIsCalibrated == 1)
            {
                cv::Mat cv_image_calib = cv_image_original;

                cv::line(cv_image_calib, Point(320 - top_x,     360 - top_y),    Point(320 + top_x,     360 - top_y),       Scalar(0, 0, 255), 1);
                cv::line(cv_image_calib, Point(320 - bottom_x,  240 + bottom_y), Point(320 + bottom_x,  240 + bottom_y),    Scalar(0, 0, 255), 1);
                cv::line(cv_image_calib, Point(320 + bottom_x,  240 + bottom_y), Point(320 + top_x,     360 - top_y),       Scalar(0, 0, 255), 1);
                cv::line(cv_image_calib, Point(320 - bottom_x,  240 + bottom_y), Point(320 - top_x,     360 - top_y),       Scalar(0, 0, 255), 1);

                cv::circle(cv_image_calib, Point2f(320 - top_x, 360 - top_y), 5, Scalar(0, 0, 255), 3);
                cv::circle(cv_image_calib, Point2f(320 + top_x, 360 - top_y), 5, Scalar(0, 0, 255), 3);
                cv::circle(cv_image_calib, Point2f(320 + bottom_x, 240 + bottom_y), 5, Scalar(0, 0, 255), 3);
                cv::circle(cv_image_calib, Point2f(320 - bottom_x, 320 - bottom_x), 5, Scalar(0, 0, 255), 3);


                //PublishMat(cv_image_calib, rcameraimageprojection_calibrated_image);
                //ROS_PUBLISH(rcameraimageprojection_pubcali, rcameraimageprojection_calibrated_image);

                // copy original image to use for cablibration
                //     cv_image_calib = np.copy(cv_image_original)

                //     # draw lines to help setting homography variables
                //     cv_image_calib = cv2.line(cv_image_calib, (160 - top_x, 180 - top_y), (160 + top_x, 180 - top_y), (0, 0, 255), 1)
                //     cv_image_calib = cv2.line(cv_image_calib, (160 - bottom_x, 120 + bottom_y), (160 + bottom_x, 120 + bottom_y), (0, 0, 255), 1)
                //     cv_image_calib = cv2.line(cv_image_calib, (160 + bottom_x, 120 + bottom_y), (160 + top_x, 180 - top_y), (0, 0, 255), 1)
                //     cv_image_calib = cv2.line(cv_image_calib, (160 - bottom_x, 120 + bottom_y), (160 - top_x, 180 - top_y), (0, 0, 255), 1)

                //     if self.pub_image_type == "compressed":
                //         # publishes calibration image in compressed type
                //         self.pub_image_calib.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_calib, "jpg"))

                //     elif self.pub_image_type == "raw":
                //         # publishes calibration image in raw type
                //         self.pub_image_calib.publish(self.cvBridge.cv2_to_imgmsg(cv_image_calib, "bgr8"))

            }

            // # adding Gaussian blur to the image of original
            // cv_image_original = cv2.GaussianBlur(cv_image_original, (5, 5), 0)

            resize(cv_image_original, cv_image_original, Size(1000, 600), INTER_LINEAR);
    
            // ## homography transform process
            // # selecting 4 points from the original image
            // pts_src = np.array([ [160 - top_x, 180 - top_y], 
            //                      [160 + top_x, 180 - top_y], 
            //                      [160 + bottom_x, 120 + bottom_y], 
            //                      [160 - bottom_x, 160 - bottom_x]])

            //autorace 2020
            //pts_src = np.array([[160 - top_x, 180 - top_y], [160 + top_x, 180 - top_y], [160 + bottom_x, 120 + bottom_y], [160 - bottom_x, 120 + bottom_y]])

            double factor_width = 1000.0 / 640.0;
            double factor_height = 600.0 / 480.0;
            vector<Point2f> pts_src;
	        pts_src.push_back(Point2f(factor_width * (320 - top_x),     factor_height * (360 - top_y)));
	        pts_src.push_back(Point2f(factor_width * (320 + top_x),     factor_height * (360 - top_y)));
	        pts_src.push_back(Point2f(factor_width * (320 + bottom_x),  factor_height * (240 + bottom_y)));
	        pts_src.push_back(Point2f(factor_width * (320 - bottom_x),  factor_height * (240 + bottom_y)));


            // # selecting 4 points from image that will be transformed
            // pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])
            

      	    vector<Point2f> pts_dst;
	        pts_dst.push_back(Point2f(200, 0));
	        pts_dst.push_back(Point2f(800, 0));
	        pts_dst.push_back(Point2f(800, 600));
	        pts_dst.push_back(Point2f(200, 600));

            // # finding homography matrix
            // h, status = cv2.findHomography(pts_src, pts_dst)

            Mat h = cv::findHomography(pts_src, pts_dst);

            std::cout << h << std::endl;

            // # homography process
            // cv_image_homography = cv2.warpPerspective(cv_image_original, h, (1000, 600))

            cv::Mat cv_image_homography;

            cv::warpPerspective(cv_image_original, cv_image_homography, h, Size(1000, 600));

            // # fill the empty space with black triangles on left and right side of bottom
            // triangle1 = np.array([[0, 599], [0, 340], [200, 599]], np.int32)

            vector<Point> triangle1;
            triangle1.push_back(Point(0,500));
            triangle1.push_back(Point(0,340));
            triangle1.push_back(Point(200,599));

            // triangle2 = np.array([[999, 599], [999, 340], [799, 599]], np.int32)

            vector<Point> triangle2;
            triangle2.push_back(Point(999,599));
            triangle2.push_back(Point(999,340));
            triangle2.push_back(Point(799,599));

            // black = (0, 0, 0)
            // white = (255, 255, 255)
            // cv_image_homography = cv2.fillPoly(cv_image_homography, [triangle1, triangle2], black)

            Scalar fillColor(0,0,0);

            vector<Point> intersectionPolygon;
            //float intersectArea = intersectConvexConvex(triangle1, triangle2, intersectionPolygon, true);
            //fillPoly(cv_image_homography, std::vector<std::vector<cv::Point>>{triangle1, triangle2}, fillColor);

            // if self.pub_image_type == "compressed":
            //     # publishes ground-project image in compressed type
            //     self.pub_image_projected.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_homography, "jpg"))

            // elif self.pub_image_type == "raw":
            //     # publishes ground-project image in raw type
            //     self.pub_image_projected.publish(self.cvBridge.cv2_to_imgmsg(cv_image_homography, "bgr8"))


            PublishMat(cv_image_homography, rcameraimageprojection_output_image);
            ROS_PUBLISH(rcameraimageprojection_pubout, rcameraimageprojection_output_image);
            
		}



	return;
}
