#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include "referee_pkg/srv/hit_armor.hpp"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

struct result{
  float yaw;
  float pitch;
  float roll;
};

class Shooter : public rclcpp::Node
{
public:
  Shooter():Node("shooter_serve"){
    shooter_service_ = this->create_service<referee_pkg::srv::HitArmor>(
        "/referee/hit_armor", bind(&Shooter::shoot_callback, this, placeholders::_1, placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "服务启动");
  }
private:
  const double v0 = 1.5;  // 增加初速度
  const double v0_sq = v0 * v0;
  const double v0_quad = v0_sq * v0_sq;
  
  Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 554.383, 0, 320,
                      0, 554.383, 240,  // 修正中心点，假设图像高度480
                      0, 0, 1);
  
  // 装甲板的世界坐标（单位：米）
  vector<cv::Point3f> worldPoints = {
      {-0.1, -0.05, 0}, // 左下
      {0.1, -0.05, 0},  // 右下
      {0.1, 0.05, 0},   // 右上
      {-0.1, 0.05, 0}   // 左上
  };
  
  rclcpp::Service<referee_pkg::srv::HitArmor>::SharedPtr shooter_service_;
  
  void shoot_callback(const shared_ptr<referee_pkg::srv::HitArmor::Request> request, 
                     shared_ptr<referee_pkg::srv::HitArmor::Response> response);
  
  bool solveTargetPosition(vector<Point2f> &imagePoints, Point3f &targetpoint);
  result calculateangles(Point3f center, double g);
};

void Shooter::shoot_callback(const shared_ptr<referee_pkg::srv::HitArmor::Request> request, 
                            shared_ptr<referee_pkg::srv::HitArmor::Response> response)
{
  // 从请求中获取信息
  auto &modelpoint = request->modelpoint;
  float g = request->g;
  auto &header = request->header;
  //放点
  vector<Point2f> points;
  for (auto p:modelpoint){
    points.push_back(Point2f(p.x,p.y));
  }
  
  // 解算目标位置
  Point3f targetpoint;
  if (!solveTargetPosition(points, targetpoint)) {
    RCLCPP_ERROR(this->get_logger(), "目标位置解算失败");
    response->yaw = 0;
    response->pitch = 0;
    response->roll = 0;
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "目标位置: (%.3f, %.3f, %.3f)", 
              targetpoint.x, targetpoint.y, targetpoint.z);
  
  // 计算欧拉角
  result result1 = calculateangles(targetpoint, g);
  response->yaw = result1.yaw;
  response->pitch = result1.pitch;
  response->roll = result1.roll;
  RCLCPP_INFO(this->get_logger(), "角度: yaw=%.3f, pitch=%.3f, roll=%.3f", 
              result1.yaw, result1.pitch, result1.roll);
}

bool Shooter::solveTargetPosition(vector<Point2f> &imagePoints, Point3f &targetpoint) {
  Mat rvec, tvec;
  Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
  
  if (imagePoints.size() != 4) {
    RCLCPP_ERROR(this->get_logger(), "需要4个点，但得到 %zu 个点", imagePoints.size());
    return false;
  }
  
  // 使用solvePnP解算位姿
  bool flag = solvePnP(worldPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
  if (!flag) {
    RCLCPP_ERROR(this->get_logger(), "solvePnP 失败");
    return false;
  }
  
  // tvec就是相机坐标系下的目标位置
  targetpoint.x = tvec.at<double>(0);
  targetpoint.y = tvec.at<double>(1); 
  targetpoint.z = tvec.at<double>(2);
  
  return true;
}

result Shooter::calculateangles(Point3f center, double g){
  result result1;
  double target_x = center.x;
  double target_y = center.y;
  double target_z = center.z;
  
  // 计算水平距离
  double distance = sqrt(target_x * target_x + target_y * target_y);
  
  RCLCPP_INFO(this->get_logger(), "水平距离: %.3f, 高度: %.3f", distance, target_z);
  
  // 计算偏航角
  result1.yaw = atan2(target_y, target_x);
  
  // 弹道计算 - 使用抛物线运动方程
  // 公式: target_z = v0*sin(pitch)*t - 0.5*g*t^2
  // 同时: distance = v0*cos(pitch)*t
  // 消去t得到: target_z = distance*tan(pitch) - (g*distance^2)/(2*v0^2*cos(pitch)^2)
  
  double tan_pitch = (target_z + (g * distance * distance) / (2 * v0_sq)) / distance;
  result1.pitch = atan(tan_pitch);
  result1.roll = 0.0;
  
  return result1;
}

int main(int argc,char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<Shooter>());
  rclcpp::shutdown();
  return 0;
}