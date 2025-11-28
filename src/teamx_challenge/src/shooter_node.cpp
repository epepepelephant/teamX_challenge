#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include "referee_pkg/srv/hit_armor.hpp"
#include "referee_pkg/msg/multi_object.hpp"  
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
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
    getvision_ = this->create_subscription<referee_pkg::msg::MultiObject>(
        "/vision/target", 10,bind(&Shooter::vision_callback, this, std::placeholders::_1));
    shooter_service_ = this->create_service<referee_pkg::srv::HitArmor>(
        "/referee/hit_arror", bind(&Shooter::shoot_callback, this, placeholders::_1, placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "服务启动");
  }

private:
  vector<Point2f> img_points;
  std_msgs::msg::Header img_header;
  const double v0 = 15.0; // 增加初速度
  const double v0_sq = v0 * v0;
  const double v0_quad = v0_sq * v0_sq;
  const double height_set = 0.222;

  Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 554.383, 0, 320,
                      0, 554.383, 320,  
                      0, 0, 1);

  // 装甲板的世界坐标（单位：米）

  rclcpp::Subscription<referee_pkg::msg::MultiObject>::SharedPtr getvision_;
  rclcpp::Service<referee_pkg::srv::HitArmor>::SharedPtr shooter_service_;

  void vision_callback(const referee_pkg::msg::MultiObject::SharedPtr msg);

  void shoot_callback(const shared_ptr<referee_pkg::srv::HitArmor::Request> request,
                      shared_ptr<referee_pkg::srv::HitArmor::Response> response);
  //获取坐标
  bool make2DTO3D(vector<Point3f> &armorpoints, Point3f &targetpoint);
  //计算角度
  result calculateangles(Point3f center, double g);
};

void Shooter::vision_callback(const referee_pkg::msg::MultiObject::SharedPtr msg){
  img_header = msg->header;
  img_points.clear();
  for(const auto&obj:msg->objects){
    for (int i = 0; i < 4;i++){
      img_points.push_back(Point2f(obj.corners[i].x, obj.corners[i].y));
       RCLCPP_INFO(this->get_logger(), "目标在图像位置: (%.3f, %.3f)", 
              obj.corners[i].x, obj.corners[i].y);
    }
  }
    RCLCPP_INFO(this->get_logger(), "接收到视觉数据");
}


void Shooter::shoot_callback(const shared_ptr<referee_pkg::srv::HitArmor::Request> request, 
                            shared_ptr<referee_pkg::srv::HitArmor::Response> response)
{
  // 从请求中获取信息
  auto &modelpoint = request->modelpoint;
  float g = request->g;
  auto &header = request->header;
  // 放点
  vector<Point3f> armorpoints;
    armorpoints.push_back(Point3f(modelpoint[3].x, modelpoint[3].z, 0.0));  // 左上
    armorpoints.push_back(Point3f(modelpoint[2].x, modelpoint[2].z, 0.0));  // 右上  
    armorpoints.push_back(Point3f(modelpoint[1].x, modelpoint[1].z, 0.0));  // 右下
    armorpoints.push_back(Point3f(modelpoint[0].x, modelpoint[0].z, 0.0));  // 左下
    cout << armorpoints << endl;
    Point3f targetpoint;

    // 解算目标位置
    if (!make2DTO3D(armorpoints, targetpoint))
    {
      RCLCPP_ERROR(this->get_logger(), "目标位置解算失败");
      response->yaw = 0;
      response->pitch = 0;
      response->roll = 0;
      return;
    }
  
  RCLCPP_INFO(this->get_logger(), "目标位置: (%.3f, %.3f, %.3f)", 
              targetpoint.x, targetpoint.y, targetpoint.z);
  
  // 计算欧拉角
  result res = calculateangles(targetpoint, g);
  response->yaw = res.yaw;
  response->pitch = res.pitch;
  response->roll = res.roll;
  RCLCPP_INFO(this->get_logger(), "角度: yaw=%.3f, pitch=%.3f, roll=%.3f", 
              res.yaw, res.pitch, res.roll);
}

bool Shooter::make2DTO3D(vector<Point3f> &armorpoints, Point3f &targetpoint) {
  Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
  Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
  
  if (img_points.size() != 4) {
    RCLCPP_ERROR(this->get_logger(), "需要4个点，但得到 %zu 个点", img_points.size());
    return false;
  }
  
  // 使用solvePnP解算位姿
  bool flag = solvePnP(armorpoints, img_points, cameraMatrix, distCoeffs, rvec, tvec);
  if (!flag) {
    RCLCPP_ERROR(this->get_logger(), "solvePnP 失败");
    return false;
  }
  
  // tvec就是相机坐标系下的目标位置
  targetpoint.x = tvec.at<double>(0);
  targetpoint.z = tvec.at<double>(1); 
  targetpoint.y = tvec.at<double>(2);
  
  return true;
}

// struct result{
//   float yaw;
//   float pitch;
//   float roll;
// };
//result类用于存放结果
result Shooter::calculateangles(Point3f center, double g){
  result res;
  double target_x = center.x;
  double target_y = center.y;
  double target_z = center.z;
  
  // 计算水平距离 R
  double R = sqrt(target_x * target_x + target_y * target_y);
  double height_diff = -target_z-height_set;
  RCLCPP_INFO(this->get_logger(), "水平距离: %.3f, 高度: %.3f", R, height_diff);
  
  // 计算偏航角 φ = arctan2(y_t, x_t)
  res.yaw = atan2(target_y, target_x);
  
  // 计算判别式 Δ = v0^4 - g(gR^2 + 2v0^2 z_t)
  // 假设 v0_sq 是 v0 的平方
  double Delta = v0_quad - g * (g * R * R + 2 * v0_sq * height_diff);
  
  // 检查判别式是否有效
  if (Delta < 0) {
    RCLCPP_ERROR(this->get_logger(), "无实数解，判别式 Delta = %.3f < 0", Delta);
    // 返回默认值或抛出异常
    res.pitch = 0.0;
    res.roll = 0.0;
    return res;
  }
  
  // 计算俯仰角 θ = arctan[(v0^2 ± sqrt(Δ)) / (gR)]
  // 这里选择正号解（通常对应较小的发射角）
  double sqrt_Delta = sqrt(Delta);
  double tan_thetahlow = (v0_sq - sqrt_Delta) / (g * R);
  double tan_thetahhigh = (v0_sq + sqrt_Delta) / (g * R);
  double tan_theta = (v0_sq - sqrt_Delta > 0) ? tan_thetahlow : tan_thetahhigh;
  res.pitch = atan(tan_theta);

  res.roll = 0.0;
  
  return res;
}
int main(int argc,char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<Shooter>());
  rclcpp::shutdown();
  return 0;
}