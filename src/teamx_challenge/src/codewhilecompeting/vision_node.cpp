#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <referee_pkg/msg/object.hpp>
#include <referee_pkg/msg/race_stage.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <unordered_map>
#include "sensor_msgs/msg/image.hpp"
#include<vector>
#include<string>
#include<algorithm>
using std::vector, std::abs,std::string, std::cin, std::cout, std::endl, std::to_string, std::make_shared, std::unordered_map;
using namespace rclcpp;
using namespace cv;

struct Red_Ring{
    Point2f pts_outter[4];
    Point2f pts_inner[4];
};
struct Arrow{
    Point2f pts[4];
};
struct HSVRange
{
    cv::Scalar lower; // HSV下限
    cv::Scalar upper; // HSV上限
};

Mat kernel_1 = getStructuringElement(MORPH_RECT, Size(1, 1));
Mat kernel_3 = getStructuringElement(MORPH_RECT, Size(3, 3));
Mat kernel_5= getStructuringElement(MORPH_RECT, Size(5,5));
Mat kernel_7= getStructuringElement(MORPH_RECT, Size(7,7));
Mat kernel_9= getStructuringElement(MORPH_RECT, Size(9,9));
Mat kernel_round= getStructuringElement(MORPH_ELLIPSE, Size(1,1));
Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));  // 可根据箭头粗细调整
unordered_map<std::string, std::vector<HSVRange>> color_hsv_map = {
    {"red", {  // 红色
        {cv::Scalar(0, 120, 70),  cv::Scalar(10, 255, 255)},
        {cv::Scalar(170, 120, 70), cv::Scalar(179, 255, 255)}
    }},
    {"green",  {{cv::Scalar(35, 120, 70),  cv::Scalar(77, 255, 255)}}},  
    {"blue",   {{cv::Scalar(100, 120, 70), cv::Scalar(130, 255, 255)}}},
    {"yellow", {{cv::Scalar(20, 120, 70),  cv::Scalar(34, 255, 255)}}},
    {"cyan",   {{cv::Scalar(80, 120, 70),  cv::Scalar(99, 255, 255)}}},
    {"purple", {{cv::Scalar(131, 120, 70), cv::Scalar(169, 255, 255)}}}
};
class TestNode : public rclcpp::Node {
    public:
    TestNode(string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "Initializing TestNode");
    Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        bind(&TestNode::callback_camera, this, std::placeholders::_1));
    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);
    Race_sub = this->create_subscription<referee_pkg::msg::RaceStage>(
        "/referee/race_stage", 10,
        bind(&TestNode::callback_stage, this, std::placeholders::_1));
    namedWindow("Detection Result", WINDOW_AUTOSIZE);

    RCLCPP_INFO(this->get_logger(), "TestNode initialized successfully");
  }
    ~TestNode() { destroyWindow("Detection Result"); }
    private:
    Mat image;//从摄像头上得到的图像
    Mat hsv;//hsv处理后的图像
    Mat result_image;//最终图像
    vector<vector<Point2f>> sphere_points_list;
    vector<Red_Ring> red_ring_list;
    vector<Arrow> arrow_list;
    Mat kernel_arror = getStructuringElement(MORPH_RECT, Size(5,5));

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
    rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
    rclcpp::Subscription<referee_pkg::msg::RaceStage>::SharedPtr Race_sub;

//回调函数
    void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);
    void callback_stage(referee_pkg::msg::RaceStage::SharedPtr msg);
//图像转换函数
    Mat convertImage(const sensor_msgs::msg::Image::SharedPtr &msg);
    void preprocess(string colortype, vector<vector<Point>> &contours, Mat kernel_out,Mat kernel_in);
    void preprocessarrow(string colortype, vector<vector<Point>> &contours, Mat kernel_out,Mat kernel_in);
//球形识别函数
    vector<Point2f> calculateStableSpherePoints(const Point2f &center, float radius);
// 找红色球形
    void detect_red_ring(vector<vector<Point>> &contours, Mat &result_image);
//找箭头
    void find_arrow(vector<vector<Point>> &contours, Mat &result_image);
//消息发布函数
    void publishmsg(const sensor_msgs::msg::Image::SharedPtr &msg);
    double sovledis(Point2f a, Point2f b);
};
void TestNode::callback_stage(referee_pkg::msg::RaceStage::SharedPtr msg)
{
    try {
      RCLCPP_INFO(this->get_logger(),"目前是第%d阶段",msg->stage);
    }catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
}
void TestNode::callback_camera(sensor_msgs::msg::Image::SharedPtr msg){
    try{
        red_ring_list.clear();
        image = convertImage(msg);
      if (image.empty())
      {
        RCLCPP_WARN(this->get_logger(), "没有图像啊");
        return;
      }
      result_image = image.clone();
        //提取hsv图像
      cvtColor(image, hsv, COLOR_BGR2HSV);
      // 定义轮廓
      vector<vector<Point>>ring_red_contours;
      vector<vector<Point>> arrow_contours;

      preprocess("red", ring_red_contours, kernel_round,kernel_round);
      preprocessarrow("red", arrow_contours, kernel_3,kernel_3);
      find_arrow(arrow_contours,result_image);
      detect_red_ring(ring_red_contours, result_image);
       imshow("Detection Result", result_image);
        waitKey(1);
      publishmsg(msg);
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
}

Mat TestNode::convertImage(const sensor_msgs::msg::Image::SharedPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    if (msg->encoding == "rgb8" || msg->encoding == "R8G8B8") {
        Mat image(msg->height, msg->width, CV_8UC3,
                      const_cast<unsigned char *>(msg->data.data()));
        Mat bgr_image;
        cvtColor(image, bgr_image, COLOR_RGB2BGR);
        cv_ptr = std::make_shared<cv_bridge::CvImage>();
        cv_ptr->header = msg->header; // 保留时间戳
        cv_ptr->encoding = "bgr8";
        cv_ptr->image = bgr_image;
    } else {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    return cv_ptr->image;
}

void TestNode::preprocess(string colortype,vector<vector<Point>>&contours,Mat kernel_out,Mat kernel_in){
  Mat maskafter=Mat::zeros(hsv.size(), CV_8UC1);;
  auto iter = color_hsv_map.find(colortype);
  const vector<HSVRange>& ranges = iter ->second;
    for (const auto& range : ranges) {
        Mat single_mask;
        inRange(hsv, range.lower, range.upper, single_mask);  // 单次筛选
        bitwise_or(maskafter, single_mask, maskafter);        // 合并掩码
    }
  morphologyEx(maskafter, maskafter, MORPH_CLOSE, kernel_in);
  morphologyEx(maskafter, maskafter, MORPH_CLOSE, kernel_in);
  morphologyEx(maskafter, maskafter, MORPH_CLOSE, kernel_in);

  findContours(maskafter, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
}
void TestNode::preprocessarrow(string colortype,vector<vector<Point>>&contours,Mat kernel_out,Mat kernel_in){
  Mat maskafter=Mat::zeros(hsv.size(), CV_8UC1);;
  auto iter = color_hsv_map.find(colortype);
  const vector<HSVRange>& ranges = iter ->second;
    for (const auto& range : ranges) {
        Mat single_mask;
        inRange(hsv, range.lower, range.upper, single_mask);  // 单次筛选
        bitwise_or(maskafter, single_mask, maskafter);        // 合并掩码
    }
    dilate(maskafter,maskafter,kernel_3);
    morphologyEx(maskafter, maskafter, MORPH_OPEN, kernel_out);
    morphologyEx(maskafter, maskafter, MORPH_CLOSE, kernel_in);


    
    imshow("11", maskafter);
    waitKey(1);
    findContours(maskafter, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
}
void TestNode::detect_red_ring(vector<vector<Point>> &contours, Mat &result_image){
    Red_Ring new_red_ring;
    vector<float> rs;
    for (int i = 0; i < contours.size(); i++) 
    {
        double area = contourArea(contours[i]);
        if (area < 500)
          continue;

        float new_r = 0;
        Point2f center;
        minEnclosingCircle(contours[i],center,new_r);
        float inner_r = sqrt((CV_PI*new_r*new_r-4*area)/CV_PI);

        // rs.push_back(new_r);
        // 计算圆形度
        double perimeter = arcLength(contours[i], true);
        double circularity = 4 * CV_PI * area / (perimeter * perimeter);
        vector<Point2f> sphere_points_outter = calculateStableSpherePoints(center, new_r );
        vector<Point2f> sphere_points_inner = calculateStableSpherePoints(center, inner_r);
        // cout<< sphere_points_outter<<endl;
        // cout<< sphere_points_inner<<endl;
        for (int k =0;k<4;k++){
        new_red_ring.pts_outter[k]=sphere_points_outter[k];
        new_red_ring.pts_inner[k]=sphere_points_inner[k];
        }
    
    red_ring_list.push_back(new_red_ring);
    }
}
void TestNode::find_arrow(vector<vector<Point>> &contours, Mat &result_image){
    for (int i = 0; i < contours.size();i++){
        Arrow new_arrow;
        double area = contourArea(contours[i]);
        RotatedRect rot_rect = minAreaRect(contours[i]);

        // cout << triangle << endl;
        if (area > 4000 || area < 100)
            continue;
        Point2f new_pts[4];
        rot_rect.points(new_pts);
        vector<Point2f> pts(new_pts, new_pts + 4);
        Point2f centerPt(0.f, 0.f); // 通过平均值找中点
        for (auto &p : new_pts) centerPt += p;
        centerPt *= (1.0f / 4.0f);
        //按x坐标排序，分成左右两组
      sort(pts.begin(), pts.end(), [](const Point2f &a, const Point2f &b) {
         return a.x < b.x;
      });

      //左边两个点按y从大到小排序（左下在前）
      sort(pts.begin(), pts.begin()+2, [](const Point2f &a, const Point2f &b) {
         return a.y > b.y;  // 大的y在前 = 下面
      });

      //右边两个点按y从小到大排序（右上在前）
      sort(pts.begin()+2, pts.end(), [](const Point2f &a, const Point2f &b) {
          return a.y > b.y;  // 小的y在前 = 上面
      });
      vector<Point2f> ordered = {pts[1], pts[3], pts[2], pts[0]};
      for (int k = 0; k < 4;k++){
        new_arrow.pts[k] = ordered[k];
      }
      vector<Point> poly;
        for (auto &p : ordered) {
          poly.push_back(Point(cvRound(p.x), cvRound(p.y)));

        }
        polylines(result_image, poly, true, Scalar(0,0,255), 2);
        putText(result_image, to_string(555), Point(cvRound(centerPt.x)-20, cvRound(centerPt.y)-20),
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255), 2);
        for (int k = 0; k < 4; ++k) {
          circle(result_image, poly[k], 6, Scalar(0,255,255), -1);
          putText(result_image, to_string(k+1), poly[k] + Point(8,-8),
                      FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,255,255), 2);
        }
      arrow_list.push_back(new_arrow);
        }
        
}
vector<Point2f> TestNode::calculateStableSpherePoints(const Point2f &center,float radius) {
    vector<Point2f> points;

  // 简单稳定的几何计算，避免漂移
  // 左、下、右、上
  points.push_back(Point2f(center.x - radius, center.y));  // 左点 (1)
  points.push_back(Point2f(center.x, center.y + radius));  // 下点 (2)
  points.push_back(Point2f(center.x + radius, center.y));  // 右点 (3)
  points.push_back(Point2f(center.x, center.y - radius));  // 上点 (4)

  return points;
}

void TestNode::publishmsg(const sensor_msgs::msg::Image::SharedPtr &msg){
    referee_pkg::msg::MultiObject msg_object;
    msg_object.header = msg->header;
    msg_object.num_objects = red_ring_list.size()*2+arrow_list.size();
    vector<string> types = {"Ring_red","arrow"};
    for (int k = 0; k < red_ring_list.size();k++){
        referee_pkg::msg::Object obj;
        obj.target_type = types[0];
        Red_Ring p = red_ring_list[k];
        geometry_msgs::msg::Point new_corner;
        for (int i = 0; i < 4; i++)
        {
            new_corner.x=p.pts_outter[i].x;
            new_corner.y = p.pts_outter[i].y;
            new_corner.z = 0.0;
            obj.corners.push_back(new_corner);
        }
        msg_object.objects.push_back(obj);
    }
        for (int k = 0; k < red_ring_list.size();k++){
        referee_pkg::msg::Object obj;
        obj.target_type = types[0];
        Red_Ring p = red_ring_list[k];
        geometry_msgs::msg::Point new_corner;
        for (int i = 0; i < 4; i++)
        {
            new_corner.x=p.pts_inner[i].x;
            new_corner.y = p.pts_inner[i].y;
            new_corner.z = 0.0;
            obj.corners.push_back(new_corner);
        }
        msg_object.objects.push_back(obj);
    }
    for (int k = 0; k < arrow_list.size();k++){
        referee_pkg::msg::Object obj;
        obj.target_type = types[1];
        Arrow p = arrow_list[k];
        geometry_msgs::msg::Point new_corner;
        for (int i = 0; i < 4; i++)
        {
            new_corner.x=p.pts[i].x;
            new_corner.y = p.pts[i].y;
            new_corner.z = 0.0;
            obj.corners.push_back(new_corner);
        }
        msg_object.objects.push_back(obj);
    }
    Target_pub->publish(msg_object);
}
double TestNode::sovledis(Point2f a, Point2f b){
    double x = a.x - b.x;
    double y = a.y - b.y;
    return sqrt(x * x + y * y);
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = make_shared<TestNode>("TestNode");
  RCLCPP_INFO(node->get_logger(), "Starting TestNode");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
