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
    //新建服务
    shooter_service_ = this->create_service<referee_pkg::srv::HitArmor>(
        "/referee/hit_armor", bind(&Shooter::shoot_callback, this, placeholders::_1, placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "服务启动");
  }
private:

  const double v0 = 1.5;
  Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 554.383, 0, 320,
                      0, 554.383, 320,
                      0, 0, 1);
  vector<cv::Point3f> worldPoints = {
      {-0.1, -0.05, 0}, // 左下
      {0.1, -0.05, 0},  // 右下
      {0.1, 0.05, 0},   // 右上
      {-0.1, 0.05, 0}   // 左上
  };
  rclcpp::Service<referee_pkg::srv::HitArmor>::SharedPtr shooter_service_;
  void shoot_callback(const shared_ptr<referee_pkg::srv::HitArmor::Request> request, shared_ptr<referee_pkg::srv::HitArmor::Response> response);
  vector<cv::Point3f> make2DTO3D(vector<Point2f> &point, vector<Point3f> &worldPoints);
  result calculateangles(Point3f center, double g);
};

void Shooter::shoot_callback(const shared_ptr<referee_pkg::srv::HitArmor::Request> request, shared_ptr<referee_pkg::srv::HitArmor::Response> response)
    {
      // 从请求中获取信息
      auto &modelpoint = request->modelpoint;
      auto &header = request->header;
      float g = request->g;
      vector<Point2f> points;
      for (auto p:modelpoint){
        points.push_back(Point2f(p.x,p.y));
      }
        // 把二维的点转化为三维的点
      make2DTO3D(points, worldPoints);

      //
      //计算中心点
      float x = 0.0f;
      float y = 0.0f;
      float z = 0.0f;
      for (Point3f point : worldPoints)
      {
        x += point.x;
        y += point.y;
        z += point.z;
      }
      x /= worldPoints.size();
      y /= worldPoints.size();
      z /= worldPoints.size();
      Point3f center = Point3f(x, y, z);

      // 计算欧拉角
      result result1=calculateangles(center,g);
      response->yaw = result1.yaw;
      response->pitch = result1.pitch;
      response->roll = result1.roll;
      RCLCPP_INFO(this->get_logger(), "yaw=%.2f,pitch=%.2f,roll=%.2f",result1.yaw,result1.pitch,result1.roll);
    }
  
vector<cv::Point3f> Shooter::make2DTO3D(vector<Point2f> &point, vector<Point3f> &worldPoints){
  Mat rvec, tvec;
  //畸变系数
  Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
   if (worldPoints.size() != 4 || point.size() != 4) {
    RCLCPP_ERROR(this->get_logger(), "Point count mismatch: image=%zu, model=%zu", 
                 worldPoints.size(), point.size());
  }
  solvePnP(worldPoints, point, cameraMatrix, distCoeffs, rvec, tvec);
  //把旋转矩阵换为旋转向量
  Mat rotationMatrix;
  Rodrigues(rvec, rotationMatrix);
  //
  vector<Point3f> camerapoints;
  for(auto&worldPoint:worldPoints){
    Mat worldMat = (Mat_<double>(3, 1) << worldPoint.x, worldPoint.y, worldPoint.z);
    Mat cameraMat = rotationMatrix * worldMat + tvec;
    camerapoints.emplace_back(cameraMat.at<double>(0),
                              cameraMat.at<double>(1),
                              cameraMat.at<double>(2));
  }
  return camerapoints;
}

result Shooter::calculateangles(Point3f center,double g){
  result result1;
  double target_x = center.x;
  double target_y = center.y;
  double target_z = center.z;
  double distance = sqrt(target_x * target_x + target_y * target_y);
  result1.yaw = atan2(target_y, target_x);
  double t = distance / v0;
  double delta = 1 - (2 * g / (v0 * v0)) * (target_z + t);
  if (delta<0){
    result1.yaw = 0;
    result1.roll = 0;
    result1.pitch = 0;
    return result1;
  }
  double sqrt_delta = sqrt(delta);
  double factor = (v0 * v0) / (g * distance);
  result1.pitch = (1 - sqrt_delta > 0) ? atan(factor * (1 - sqrt_delta)): atan(factor * (1 + sqrt_delta));
  result1.roll = 0.0;
  return result1;
}

int main(int argc,char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<Shooter>());
  rclcpp::shutdown();
  return 0;
}
