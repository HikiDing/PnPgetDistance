#ifndef _PNPFORRM_
#define _PNPFORRM_

#include<opencv2/opencv.hpp>
#include<bits/stdc++.h>

namespace PnpforRM
{
    class PnpforRM
    {
    private:
        /* data */
        cv::Point2f targetPoint;
        int k = 5000;
        std::vector<cv::Point3f> Points3D;
        
        double cameraMtx[9] = {1262.748030 , 0.000000 , 315.770278,
						       0.000000 , 1259.194056 , 226.760690,
						       0.000000 , 0.000000    , 1.000000}; //3*3的相机内参
        
        double distMtx[5]   =  {-0.421204,-0.237514,-0.003354,-0.001293,0.000000}; //5*1的畸变系数矩阵 

    public:
        cv::VideoCapture webcamera;
        std::vector<cv::Point2f> rectPoint;
        cv::Rect roiRect; 
        PnpforRM(/* args */);
        ~PnpforRM();

        void openWebCamera(int webID);
        void getTarget(cv::Mat srcImage);
        void changePoint();
        void getDistance();
        void getDistance(std::vector<cv::Point2f> Points2D);
        void findrect(cv::Mat srcImage);
    };

    PnpforRM::PnpforRM(/* args */)
    {
    }

    PnpforRM::~PnpforRM()
    {
    }
    void PnpforRM::openWebCamera(int webID)
    {
        webcamera.open(webID);
        webcamera.set(cv::CAP_PROP_FRAME_WIDTH,640);
        webcamera.set(cv::CAP_PROP_FRAME_HEIGHT,480);
    }

    void PnpforRM::getTarget(cv::Mat srcImage)
    {
        cv::Mat tempImg;
    	cv::Mat dstImg;
        std::vector<cv::Vec3f>pcircles;
	    std::vector<std::vector<cv::Point>> contours;
	    std::vector<cv::Vec4i> hierarchy;

    	if (srcImage.empty())
    	{
    		std::cout << "could not find srcImage" << std::endl;
    		exit(0);
    	}
        targetPoint = cv::Point2f(0,0);
        srcImage.copyTo(dstImg);
    	cv::medianBlur(srcImage,tempImg,3);
    	cv::cvtColor(tempImg,tempImg,cv::COLOR_BGR2GRAY);
        
        
        cv::HoughCircles(tempImg,pcircles,cv::HOUGH_GRADIENT,1.3,10, 200, 100,10,300);
    	for (size_t i = 0; i < pcircles.size(); i++)
    	{
            //参数定义
            cv::Point center(cvRound(pcircles[i][0]), cvRound(pcircles[i][1]));
            int radius = cvRound(pcircles[i][2]);
            //std::cout<<"x: "<<center.x << " y: " <<center.y<<"R: "<<radius<< std::endl;
            //绘制圆心
            //circle( srcImage, center, 4,cv::Scalar(0,255,250),8,6,0); // 填满
            //绘制圆轮廓
            circle(dstImg, center, radius,cv::Scalar(155,50,255),8,6,0 );
            break;
    	}
    	cv::imshow("output", dstImg);
    }
    void PnpforRM::findrect(cv::Mat srcImage)
    {
        int rmin = 114;
        int gmin = 112;
        int bmin = 23;

        int rmax = 255;
        int gmax = 255;
        int bmax = 103;
	    cv::Rect roi_rect;
	    cv::Rect roi_temp;
	    int maxArea = 0;
    
	    cv::Mat image_temp,dstimage;
	    srcImage.copyTo(image_temp);
	    srcImage.copyTo(dstimage);                           

	    cv::Scalar low = cv::Scalar(bmin,gmin,rmin);
        cv::Scalar high = cv::Scalar(bmax,gmax,rmax);
        
        std::vector<std::vector<cv::Point>> contours;
	    std::vector<cv::Vec4i> hierarchy;
	    std::vector<cv::Rect> rect_vector;
        
        cv::inRange(image_temp,low,high,image_temp);
        findContours(image_temp, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
	    for(std::vector<std::vector<cv::Point>>::iterator It = contours.begin(); It < contours.end(); It++)
	    {
	    	roi_temp = boundingRect(*(It));
	    	if (maxArea < roi_temp.area())
	    	{
	    		maxArea = roi_temp.area();
	    		roi_rect = roi_temp;
	    	}
	    }
	    cv::rectangle(dstimage,roi_rect, cv::Scalar(0, 255, 0), 3);
        this->roiRect = roi_rect;
        cv::imshow("Frame",image_temp);
	    cv::imshow("dstImage",dstimage);

    }
    void PnpforRM::changePoint()
    {
        rectPoint.clear();

        cv::Point2f leftTop  = cv::Point2f(targetPoint.x - 50 , targetPoint.y - 50);
        cv::Point2f rightTop = cv::Point2f(targetPoint.x + 50 , targetPoint.y - 50);
        cv::Point2f leftLow  = cv::Point2f(targetPoint.x - 50 , targetPoint.y + 50);
        cv::Point2f rightLow = cv::Point2f(targetPoint.x + 50 , targetPoint.y + 50);
        
        rectPoint.push_back(leftTop);
        rectPoint.push_back(rightTop);
        rectPoint.push_back(leftLow);
        rectPoint.push_back(rightLow);
    }
    
        
    void PnpforRM::getDistance() // Pinhole
    {
        int K = 5000;
        float len ;
        len  = K / (0.5 * this->roiRect.area());
        std::cout << len << std::endl;
    }   
    void PnpforRM::
    getDistance(std::vector<cv::Point2f> Points2D) // PNP
    {

        cv::Mat camera_matrix = cv::Mat(3, 3, CV_64FC1, cameraMtx);
        cv::Mat distortion_coefficients = cv::Mat(5, 1, CV_64FC1, distMtx);
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
    	//std::cout << Points2D[0] << std::endl;

        Points3D.push_back(cv::Point3f(-2.0f, -2.0f, 0));		//左xia 三维坐标的单位是毫米
	    Points3D.push_back(cv::Point3f(+2.0f, -2.0f, 0));		//右上
	    Points3D.push_back(cv::Point3f(+2.0f, +2.0f, 0));		//右下
	    //Points3D.push_back(cv::Point3f(150, 200, 0));	//P4
        Points3D.push_back(cv::Point3f(-2.0f, +2.0f, 0));	//左上

        //三种方法求解
	    solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, cv::SOLVEPNP_EPNP);	
        double rm[3][3];
        cv::Mat rotM(3, 3, CV_64FC1, rm);
        Rodrigues(rvec, rotM);
        //std::cout << rotM << std::endl;

        double EularAngles_x = atan2(rm[2][1], rm[2][2]);
        double EularAngles_y = atan2(-rm[2][0], sqrt(rm[2][0] * rm[2][0] + rm[2][2] * rm[2][2]));
        double EularAngles_z= atan2(rm[1][0], rm[0][0]);

        std::cout << "x:    " << EularAngles_x << "   y:  " << EularAngles_y << std::endl;


        //double theta_z = atan2(rm[1][0], rm[0][0])*57.2958;
        //double theta_y = atan2(-rm[2][0], sqrt(rm[2][0] * rm[2][0] + rm[2][2] * rm[2][2]))*57.2958;
        double theta_x = atan2(rm[2][1], rm[2][2])*57.2958;
        std::cout << "Height   " << theta_x << std::endl;


        //std::cout << "z:"<< theta_z << "x:"<<theta_x << "y:" <<theta_y << std::endl;
        //std::cout << tvec << std::endl; //平移矩阵，可以用来直接做距离的转换
        
        //输出3个值
        //std::cout << tvec.at<double>(0) <<":"<< tvec.at<double>(1) <<":" 
        std::cout <<tvec.at<double>(2) <<std::endl;
        //std::cout << rvec << std::endl;
        Points3D.clear();
    } 
}
#endif _PNPFORRM_