#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class pnpGetdistance : public rclcpp::Node
{
private:
  rclcpp::Parameter cameraArray,distArray;
  std::vector<double> cameraMtx;
  std::vector<double> distMtx;
  std::vector<cv::Point2d> Points2D;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_novel;

  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    Points2D.clear();
    Points2D.push_back(cv::Point2d(300,300));   //左上
    Points2D.push_back(cv::Point2d(500,300));   //右上
    Points2D.push_back(cv::Point2d(500,500));   //右下
    Points2D.push_back(cv::Point2d(300,500));   //左下

    std::cout << getDistance(Points2D) << std::endl;    
    
    RCLCPP_INFO(this->get_logger(), "'%s'", msg->data.c_str());
  };

  /*[1262.748030 , 0.000000 , 315.770278,
		 0.000000 , 1259.194056 , 226.760690,
		 0.000000 , 0.000000    , 1.000000  ] //3*3的相机内参
        
    [-0.421204,-0.237514,-0.003354,-0.001293,0.000000] //5*1的畸变系数矩阵 */

public:
  pnpGetdistance(std::string name) : Node(name)
  {
    init();
    sub_novel = this->create_subscription<std_msgs::msg::String>("say", 10, std::bind(&pnpGetdistance::topic_callback, this, _1));
  }

  bool init()
  {
    this->declare_parameter("cameraArray");
    this->declare_parameter("distArray");
    
    this->get_parameter("cameraArray",cameraArray);
    this->get_parameter("distArray",distArray);

    cameraMtx = cameraArray.as_double_array();
    distMtx   = distArray.as_double_array(); 

  }

  double getPowSum(double number)
  {
      return number*number;
  }

  double getDistance(std::vector<cv::Point2d> Points2D)
  {
    std::vector<cv::Point3f> smallArmor,bigArmor;
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_64FC1, cameraMtx.data());
    cv::Mat distortion_coefficients = cv::Mat(5, 1, CV_64FC1, distMtx.data());
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
                                                          //小装甲板
    smallArmor.push_back(cv::Point3f(-8.0f, -8.0f, 0));		//左上 三维坐标的单位是cm
	  smallArmor.push_back(cv::Point3f(+8.0f, -8.0f, 0));		//右上
	  smallArmor.push_back(cv::Point3f(+8.0f, +8.0f, 0));		//右下
    smallArmor.push_back(cv::Point3f(-8.0f, +8.0f, 0));	  //左上

	  solvePnP(smallArmor, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, cv::SOLVEPNP_EPNP);	
    
    std::cout << tvec << std::endl; //平移矩阵，可以用来直接做距离的转换
    std::cout << sqrt(getPowSum(tvec.at<double>(1)) + getPowSum(tvec.at<double>(2)) + getPowSum(tvec.at<double>(3))) << std::endl;
    smallArmor.clear();

    return  sqrt(getPowSum(tvec.at<double>(1)) + getPowSum(tvec.at<double>(2)) + getPowSum(tvec.at<double>(3)));

  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::vector<double> adc[3];

  auto node = std::make_shared<pnpGetdistance>("test");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
