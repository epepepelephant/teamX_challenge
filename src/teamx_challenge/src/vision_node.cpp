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
using std::vector, std::string, std::cin, std::cout, std::endl, std::to_string,std::make_shared,std::unordered_map;
using namespace rclcpp;
using namespace cv;
// 装甲板
struct Armor{
    RotatedRect r;
    Point2f pts[4];
    float long_side;
    float short_side;
    double area;
    float angle;
    int num = -1;
};
//装甲板上识别出来的数字的顶点（用于仿射变换）
struct numpoints{
   Point2f pts[4];
};
// 灯条
struct Light{
    RotatedRect r;
    Point2f pts[4];
    float angle;
    float long_side;
    float short_side;
};
//球体
struct Sphere{
  Point2f pts[4];
  Point2f center;
  float radius;
  float area;
};
//长方体
struct Rects{
  Point2f pts[4];
  Point2f center;
  float area;
  float width;
  float height;
};
// 跟踪目标
struct TrackedTarget{
  int id;
  Point2f center;
  float size;
  string type;
  int miss_count;//丢失帧数
};
//颜色的范围
struct HSVRange{
  cv::Scalar lower; // HSV下限
  cv::Scalar upper; // HSV上限
};
// kernel
Mat kernel_1 = getStructuringElement(MORPH_RECT, Size(1, 1));
Mat kernel_3 = getStructuringElement(MORPH_RECT, Size(3, 3));
Mat kernel_5= getStructuringElement(MORPH_RECT, Size(5,5));
Mat kernel_round= getStructuringElement(MORPH_ELLIPSE, Size(3,3));
//创建颜色的哈希表
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
    gettemplate(template_imgs);
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
   Mat mask_white;//提取白色的图像（用于数字识别）
   Mat hsv;//hsv处理后的图像
   Mat result_image;//最终图像

   // 存放装甲板和灯条的信息
   vector<Light> light_list;
   vector<Armor> armor_red_list;

   // 存放球体的信息
   vector<Sphere> sphere_list;

   // 长方体
   vector<Rects> rect_list;

   // 存放数字模板
   vector<Mat> template_imgs;
   // 数字图像的四个顶点，用于仿射变换
   vector<numpoints> num_points;
   // 仿射变换后的图像
   vector<Mat> pers_imgs;
   // 动态运动中存在的对象
   vector<TrackedTarget> tracker_targets;
   // 当前帧出现的对象
   vector<TrackedTarget> current_frame_targets;
   // 动态识别对应的id号
   int next_target_id = 1;
   // 动态识别最大允许丢失的帧数
   const int MAX_MISS_FRAMES = 3;
   // 模板匹配阈值
   const double MATCH_THRESHOLD = 0.75;

   // 订阅相机图像
   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
   rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
   rclcpp::Subscription<referee_pkg::msg::RaceStage>::SharedPtr Race_sub;

   //---------回调函数-----------//
    void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);
    void callback_stage(referee_pkg::msg::RaceStage::SharedPtr msg);

  //---------图像转换函数-----------//
    Mat convertImage(const sensor_msgs::msg::Image::SharedPtr &msg);
    void preprocess(string colortype,vector<vector<Point>>&contours,Mat kernel);
    //---------装甲板识别的相关函数-----------//

  // 灯条筛选
    void findLights(const vector<vector<Point>> &contours);
  //灯条匹配    
    void matchLights(Mat &result_img); 
  // 装甲板构造函数
    bool constructArmor(const Light &light1, const Light &light2, vector<Point2f> &armor_corners);
  //画四个顶点
    void drawarmor(Mat &result_img ,vector<Point> &armor_points,vector<Point2f> &armor_corners);

  //---------球体识别的相关函数-----------//
  // 稳定的球体点计算方法
    vector<Point2f> calculateStableSpherePoints(const Point2f &center,float radius);
  //找红色球形
    void detect_red_circle(vector<vector<Point>> &contours, Mat &result_image);

  //---------长方体识别的相关函数-----------//
  //找青色矩形
    void detect_green_rect(vector<vector<Point>> &contours, Mat &result_image);

  //---------数字识别的相关函数-----------//
  // 仿射变换
    void perspective_transformation(vector<numpoints> num_points,vector<Mat> &pers_imgs);
  //预处理数字图像
    void preprocessnumber(vector<Mat> &pers_imgs);
  //处理模板
    void gettemplate(vector<Mat>&template_imgs);
  //模板匹配
    void getnumbers(vector<Mat> &pers_imgs, Mat &result_img,vector<Mat>&template_imgs);
  //---------动态识别相关函数-----------//
  //获取当前帧装甲板
    void gettargets(Armor &armor);
  //帧间匹配
    void framematch();
  //---------消息发布函数-----------//
    void publishmsg(const sensor_msgs::msg::Image::SharedPtr &msg);
};
 //---------回调函数-----------//
void TestNode::callback_stage(referee_pkg::msg::RaceStage::SharedPtr msg) {
    try {
      RCLCPP_INFO(this->get_logger(),"目前是第%d阶段",msg->stage);
    }catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
}

void TestNode::callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      current_frame_targets.clear();
      // 图像转换
      light_list.clear();
      num_points.clear();
      rect_list.clear();
      sphere_list.clear();
      armor_red_list.clear();
      pers_imgs.clear();
      num_points.clear();

      image = convertImage(msg);
      if (image.empty())
      {
        RCLCPP_WARN(this->get_logger(), "没有图像啊");
        return;
      }
        result_image = image.clone();
        //提取hsv图像
        cvtColor(image, hsv, COLOR_BGR2HSV); 
        //定义轮廓  
        vector<vector<Point>>contours_armor_red;
        vector<vector<Point>> contours_sphere;
        vector<vector<Point>> contours_rect;
         // 小球
         preprocess("red", contours_sphere, kernel_round);
         detect_red_circle(contours_sphere, result_image);
         // 青色矩形
         preprocess("cyan", contours_rect, kernel_1);
         detect_green_rect(contours_rect, result_image);
         // 数字检测
         preprocess("red", contours_armor_red, kernel_1);
         findLights(contours_armor_red);
         matchLights(result_image);
         perspective_transformation(num_points, pers_imgs);
         preprocessnumber(pers_imgs);
         getnumbers(pers_imgs, result_image, template_imgs);
         //显示数字图像，用于调试
         for (int i = 0; i < pers_imgs.size(); i++)
         {
           imshow(to_string(i), pers_imgs[i]);
           waitKey(1);
         }
         //动态跟踪
         for (int i = 0; i < armor_red_list.size();i++){
           gettargets(armor_red_list[i]);
         }
         framematch();
          imshow("Detection Result", result_image);
          waitKey(1);
        // 创建并发布消息
        publishmsg(msg);
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}

//---------图像转换函数-----------//
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

void TestNode::preprocess(string colortype,vector<vector<Point>>&contours,Mat kernel){
  Mat maskafter=Mat::zeros(hsv.size(), CV_8UC1);;
  auto iter = color_hsv_map.find(colortype);
  const vector<HSVRange>& ranges = iter ->second;
    for (const auto& range : ranges) {
        Mat single_mask;
        inRange(hsv, range.lower, range.upper, single_mask);  // 单次筛选
        bitwise_or(maskafter, single_mask, maskafter);        // 合并掩码
    }
  morphologyEx(maskafter, maskafter, MORPH_CLOSE, kernel);
  morphologyEx(maskafter, maskafter, MORPH_OPEN, kernel);
  findContours(maskafter, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
}
//---------装甲板识别的相关函数-----------//
void TestNode::findLights(const vector<vector<Point>> &contours) {
    for (size_t i = 0; i < contours.size(); i++) {
        //最小外接矩形拟合以及灯条筛选
        RotatedRect rot_rect = minAreaRect(contours[i]);
        float width = rot_rect.size.width;
        float height = rot_rect.size.height;
        float long_side = max(width, height);
        float short_side = min(width, height);
        float aspect_ratio = long_side / short_side;
        float area_rect = long_side * short_side;
        float inital_area = contourArea(contours[i]);
        float area_ratio = inital_area / area_rect;
        float angle = rot_rect.angle;
        if(angle<45.0f)
          angle = 90.0-angle;

        //用面积，填充率，长宽比筛选灯条
        if(area_rect<5||aspect_ratio<1.5||area_ratio<0.1) continue;
        Light light;light.r=rot_rect; light.long_side=long_side; light.short_side=short_side; light.angle=angle;
        rot_rect.points(light.pts);//把旋转矩阵的顶点信息存入结构体
        light_list.push_back(light);//把light添加到light_list容器中
        //绘制灯条
        vector<Point> rect_points;
        for(int j=0;j<4;j++){
            rect_points.push_back(Point(cvRound(light.pts[j].x),cvRound(light.pts[j].y))); 
        }
        polylines(result_image, rect_points, true, Scalar(255,0,255),1);
        //用debug分析灯条为啥被过滤掉
        // RCLCPP_DEBUG(this->get_logger(),
        //               "Found light: Center(%.1f, %.1f) Long: %.1f Short: %.1f Angle: %.1f",
        //               rot_rect.center.x, rot_rect.center.y,
        //               long_side, short_side, angle);
    }
}

void TestNode::matchLights(Mat &result_img){
  vector<bool> matches(light_list.size(), true);
  for (int i = 0; i < light_list.size(); i++)
  {
    for (int j = i + 1; j < light_list.size(); j++)
    {
      Light light1 = light_list[i];
      Light light2 = light_list[j];
      // 角度筛选
      float angle_diff = abs(light1.angle - light2.angle);
      if (angle_diff > 30.0f)
        continue;
      // 高度筛选
      float max_long = max(light1.long_side, light2.long_side);
      float min_long = min(light1.long_side, light2.long_side);
      if (min_long * 2.0f < max_long)
        continue;
      // 垂直位置偏差筛选
      float y_diff = abs(light1.r.center.y - light2.r.center.y);
      float avg_long = (light1.long_side + light2.long_side) / 2.0f;
      if (y_diff > avg_long * 2.0f)
        continue;
      // 横向距离合理性筛选
      float x_diff = abs(light1.r.center.x - light2.r.center.x);
      if (x_diff < avg_long * 0.35f || x_diff > avg_long * 10.0f)
        continue;

      Armor new_armor;
      vector<Point2f> armor_corners;
      if (constructArmor(light1, light2, armor_corners) && matches[i] && matches[j])
      {
        for (int k = 0; k < 4;k++){
          new_armor.pts[k] = armor_corners[k];
        }

        RotatedRect armor_rect = minAreaRect(armor_corners);
        new_armor.r = armor_rect;
        new_armor.long_side = max(armor_rect.size.width, armor_rect.size.height);new_armor.short_side = min(armor_rect.size.width, armor_rect.size.height);new_armor.area = new_armor.long_side * new_armor.short_side;new_armor.angle = armor_rect.angle;
        armor_red_list.push_back(new_armor);
        matches[i] = false;
        matches[j] = false;
        
        vector<Point> armor_points;
        for (auto &p : new_armor.pts)
        {
          armor_points.push_back(Point(cvRound(p.x), cvRound(p.y)));

        }
        drawarmor(result_img, armor_points, armor_corners);
        //基于装甲板比例提取ROI
        Rect num_roi;
        float armor_width = abs(new_armor.pts[0].x - new_armor.pts[1].x);
        float armor_height = abs(new_armor.pts[0].y - new_armor.pts[2].y);
        Point2f centera = (new_armor.pts[0] + new_armor.pts[2]) / 2.0f;
        float roi_x = centera.x - armor_width * 0.6f;
        float roi_y = centera.y - armor_height * 1.0f;
        float roi_width = armor_width * 1.5f;
        float roi_height = armor_height * 2.0f;

        // 限制ROI边界，确保不超出图像范围
        roi_x = max(roi_x, 0.0f);
        roi_y = max(roi_y, 0.0f);
        roi_width = min(roi_width, (float)image.cols - roi_x);
        roi_height = min(roi_height, (float)image.rows - roi_y);
                
        num_roi = Rect(roi_x,roi_y,roi_width,roi_height);
        Point2f topleft;
        topleft = Point2f(centera.x - armor_width * 0.6, centera.y - armor_height * 1.0);
        Mat roi_img = image(num_roi);
        Mat hsv_1;
        // 下面提取ROI的白色区域
        cvtColor(roi_img, hsv_1, COLOR_BGR2HSV);
        inRange(hsv_1, Scalar(0, 0, 200), Scalar(180, 50, 255), mask_white);
        vector<vector<Point>> contours_white;

        findContours(mask_white, contours_white, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        
        for (int i = 0; i < contours_white.size(); i++)
        {

          Rect r = boundingRect(contours_white[i]);
          double area_contour = contourArea(contours_white[i]);
          if (area_contour < 50)
            continue; // 忽略过小噪声
          numpoints new_numpoints;
          Point2f temp_points[4];
          //在这里修改，改为armor的左上顶点（在图中的实际位置）
          temp_points[0] = Point2f(r.x, r.y)+topleft;
          temp_points[1] = Point2f(r.x+r.width, r.y)+topleft;
          temp_points[2] = Point2f(r.x, r.y+r.height)+topleft;
          temp_points[3] = Point2f(r.x+r.width, r.y+r.height)+topleft;
          for (int j = 0; j < 4; ++j)
          {
            new_numpoints.pts[j] = temp_points[j];
          }


          num_points.push_back(new_numpoints);
        }
      }
  }
}
}

bool TestNode::constructArmor(const Light &light1, const Light &light2, vector<Point2f> &armor_corners){
    const Light &left_light = (light1.r.center.x < light2.r.center.x) ? light1 : light2;
    const Light &right_light = (light1.r.center.x < light2.r.center.x) ? light2 : light1;

    // 获取左右灯条的所有顶点
    Point2f left_pts[4], right_pts[4];
    left_light.r.points(left_pts);
    right_light.r.points(right_pts);
    
    vector<Point2f> left_vec(left_pts, left_pts + 4);
    vector<Point2f> right_vec(right_pts, right_pts + 4);

    // 按x坐标排序，分成左右两组
    sort(left_vec.begin(), left_vec.end(), [](const Point2f &a, const Point2f &b) {
        return a.x < b.x;
    });
    sort(right_vec.begin(), right_vec.end(), [](const Point2f &a, const Point2f &b) {
        return a.x < b.x;
    });

    // 左灯条取x较小的两个点（外侧），右灯条取x较大的两个点（外侧）
    vector<Point2f> left_outer = {left_vec[0], left_vec[1]};
    vector<Point2f> right_outer = {right_vec[2], right_vec[3]};

    // 合并四个角点
    vector<Point2f> all_pts = {left_outer[0], left_outer[1], right_outer[0], right_outer[1]};

    // 按照您的排序逻辑：先按x分成左右，再分别按y排序
    sort(all_pts.begin(), all_pts.end(), [](const Point2f &a, const Point2f &b) {
        return a.x < b.x;
    });

    // 左边两个点按y从大到小排序（y大的在下）
    sort(all_pts.begin(), all_pts.begin() + 2, [](const Point2f &a, const Point2f &b) {
        return a.y > b.y;  // 大的y在前 = 下面
    });

    // 右边两个点按y从小到大排序（y小的在上）
    sort(all_pts.begin() + 2, all_pts.end(), [](const Point2f &a, const Point2f &b) {
        return a.y < b.y;  // 小的y在前 = 上面
    });
    armor_corners = {all_pts[0], all_pts[3], all_pts[2], all_pts[1]};

    return true;
}

void TestNode::drawarmor(Mat &result_img , vector<Point> &armor_points,vector<Point2f> &armor_corners){
    polylines(result_image, armor_points, true, Scalar(0,0,255),2);
    for (int k = 0; k < 4; k++)
    {
      circle(result_image, armor_corners[k], 6, Scalar(0, 255, 255), -1);
      circle(result_image, armor_corners[k], 6, Scalar(0, 0, 0), 2);
      putText(result_image,to_string(k+1),
                  Point(armor_corners[k].x + 10, armor_corners[k].y - 10),
                  FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);
    }
}

//---------球体识别的相关函数-----------//
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

void TestNode::detect_red_circle(vector<vector<Point>> &contours, Mat &result_image){
  Sphere new_sphere;
  for (size_t i = 0; i < contours.size(); i++)
  {
    // 面积过滤
    double area = contourArea(contours[i]);
    if (area < 500)
      continue;

    // 计算最小外接圆
    Point2f center;
    float radius = 0;
    minEnclosingCircle(contours[i], center, radius);

    // 计算圆形度
    double perimeter = arcLength(contours[i], true);
    double circularity = 4 * CV_PI * area / (perimeter * perimeter);

    if (circularity > 0.7 && radius > 15 && radius < 200)
    {
      vector<Point2f> sphere_points = calculateStableSpherePoints(center, radius);
      // 绘制检测到的球体
      circle(result_image, center, static_cast<int>(radius),
                 Scalar(0, 255, 0), 2); // 绿色圆圈
      circle(result_image, center, 3, Scalar(0, 0, 255),
                 -1); // 红色圆心

      // 绘制球体上的四个点
      vector<string> point_names = {"左", "下", "右", "上"};
      vector<Scalar> point_colors = {
          // 特征点颜色
          Scalar(255, 0, 0),   // 蓝色 - 左
          Scalar(0, 255, 0),   // 绿色 - 下
          Scalar(0, 255, 255), // 黄色 - 右
          Scalar(255, 0, 255)  // 紫色 - 上
      };
      for (int k = 0; k < 4;k++){
      new_sphere.pts[k] = sphere_points[k];
      }
      
      new_sphere.center = center;
      new_sphere.radius = radius;
      new_sphere.area = CV_PI * radius * radius;

      for (int j = 0; j < 4; j++)
      {
        circle(result_image, sphere_points[j], 6, point_colors[j], -1);
        circle(result_image, sphere_points[j], 6, Scalar(0, 0, 0), 2);

        // 标注序号
        string point_text = to_string(j + 1);
        putText(
            result_image, point_text,
            Point(sphere_points[j].x + 10, sphere_points[j].y - 10),
            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 3);
        putText(
            result_image, point_text,
            Point(sphere_points[j].x + 10, sphere_points[j].y - 10),
            FONT_HERSHEY_SIMPLEX, 0.6, point_colors[j], 2);

        // 添加到发送列表


        RCLCPP_INFO(this->get_logger(),
                    "Sphere , Point %d (%s): (%.1f, %.1f)",
                     j + 1, point_names[j].c_str(),
                    sphere_points[j].x, sphere_points[j].y);
      }

      // 显示半径信息
      string info_text = "R:" + to_string((int)radius);
      putText(
          result_image, info_text, Point(center.x - 15, center.y + 5),
          FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);
      sphere_list.push_back(new_sphere);
      RCLCPP_INFO(this->get_logger(),
                  "Found sphere: (%.1f, %.1f) R=%.1f C=%.3f", center.x,
                  center.y, radius, circularity);

    }
  }
}

//---------长方体识别的相关函数-----------//
void TestNode::detect_green_rect(vector<vector<Point>> &contours, Mat &result_image){
  for (size_t i = 0; i < contours.size(); ++i) {
      double area_contour = contourArea(contours[i]);
      if (area_contour < 15) continue; // 忽略过小噪声

      // 用最小外接旋转矩形拟合轮廓
      RotatedRect r = minAreaRect(contours[i]);
      double rect_area = abs(r.size.width * r.size.height);
      if (rect_area < 1.0) continue;
      double fillRatio = area_contour / rect_area;
      if (fillRatio < 0.2) continue; // 太稀疏的轮廓（非实心矩形）

      // 尺寸/长宽比筛选
      Rects new_rect;
      double a = r.angle;
      double w = r.size.width;
      double h = r.size.height;
      double longSide = max(w,h);
      double shortSide = max(1.0, min(w,h));
      double aspect = longSide / shortSide;


      if (longSide < 10 || shortSide < 5) continue;
      if (aspect > 10.0) continue;

      // 获取四角点并排序为 TL,TR,BR,BL
      Point2f ptsf[4];
      r.points(ptsf);
      vector<Point2f> pts(ptsf, ptsf+4);
      
      // 计算中心并分上下两组，再左右排序
      Point2f centerPt(0.f, 0.f);//通过平均值找中点
      for (auto &p : pts) centerPt += p;
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


        vector<Point2f> ordered = {pts[0], pts[2], pts[3], pts[1]}; // TL,TR,BR,BL

        // 绘制并记录
        vector<Point> poly;
        for (auto &p : ordered) {
          poly.push_back(Point(cvRound(p.x), cvRound(p.y)));

        }
        polylines(result_image, poly, true, Scalar(0,0,255), 2);
        putText(result_image, to_string(a), Point(cvRound(centerPt.x)-20, cvRound(centerPt.y)-20),
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255), 2);
        for (int k = 0; k < 4; ++k) {
          circle(result_image, poly[k], 6, Scalar(0,255,255), -1);
          putText(result_image, to_string(k+1), poly[k] + Point(8,-8),
                      FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,255,255), 2);
        }
        for (int k = 0; k < 4;k++){
          new_rect.pts[k] = ordered[k];
        }
        new_rect.center = centerPt;
        new_rect.area = area_contour;
        new_rect.width = r.size.width;
        new_rect.height = r.size.height;
        rect_list.push_back(new_rect);
       
        RCLCPP_INFO(this->get_logger(), "RotRect  area=%.1f fill=%.2f W=%.1f H=%.1f",
                    area_contour, fillRatio, r.size.width, r.size.height);
}
}

//---------数字识别的相关函数-----------//
void TestNode::perspective_transformation(vector<numpoints> num_points,vector<Mat> &pers_imgs){
  float x = 150.0;
  float y = 250.0;
  for (int i = 0; i < num_points.size(); i++)
  {
    Point2f src[4] = {num_points[i].pts[0], num_points[i].pts[1], num_points[i].pts[2], num_points[i].pts[3]};
    Point2f dst[4] = {{0.0f, 0.0f}, {x, 0.0f}, {0.0f, y}, {x, y}};
    Mat matrix = getPerspectiveTransform(src, dst);
    Mat new_img;
    warpPerspective(image, new_img, matrix, Size(x, y));
    pers_imgs.push_back(new_img);
    imshow("perspective",new_img);
    waitKey(1);
  }
}

void TestNode::preprocessnumber(vector<Mat>&pers_imgs){
  for (int i = 0; i < pers_imgs.size();i++){
    cvtColor(pers_imgs[i], pers_imgs[i], COLOR_BGR2GRAY);
    //二值化，注意阈值（红色需要筛除
    threshold(pers_imgs[i], pers_imgs[i], 127, 255, THRESH_BINARY);
    morphologyEx(pers_imgs[i], pers_imgs[i], MORPH_OPEN, kernel_3);
    morphologyEx(pers_imgs[i], pers_imgs[i], MORPH_CLOSE, kernel_3);
    // imshow("preprocessnumber", pers_imgs[i]);
    // waitKey(1);
  }
}

void TestNode::gettemplate(vector<Mat>&templaate_imgs){
  template_imgs.clear();
  for (int i = 1; i <= 5; i++)
  {
    string package_share_dir = ament_index_cpp::get_package_share_directory("teamx_challenge");
    string pathi = package_share_dir + "/numbers/" + to_string(i) + ".jpg";    Mat new_template = imread(pathi);
    cvtColor(new_template, new_template, COLOR_BGR2GRAY); // 转为1通道灰度图
    threshold(new_template, new_template, 127, 255, THRESH_BINARY); // 1通道二值图
    resize(new_template, new_template, Size(150, 250));
    template_imgs.push_back(new_template);
    // imshow(to_string(i) + "template", new_template);
    // waitKey(1);
  }
}

void TestNode::getnumbers(vector<Mat> &pers_imgs, Mat &result_img,vector<Mat> &template_imgs){
  for (int i = 0; i < pers_imgs.size(); i++)
  {
    double min_score = 1e9;
    int best_match_idx = -1;
    for (int j = 0; j < template_imgs.size();j++){
      Mat match_result;
      //归一化平方差，越相似，分数越接近0
      //match_result是匹配分数矩阵
      matchTemplate(pers_imgs[i], template_imgs[j], match_result, TM_SQDIFF_NORMED);
      double score;
      Point min_loc, max_loc;
      minMaxLoc(match_result, &score, nullptr, &min_loc, nullptr);
      if(score<min_score){
        min_score = score;
        best_match_idx = j;
      }
     
    }
    if(best_match_idx != -1 && min_score < MATCH_THRESHOLD){
        int re_number = best_match_idx + 1;
        //打印分数
        //putText(result_image, to_string(min_score), Point2f(armor_red_list[i].pts[0].x - 25, armor_red_list[i].pts[0].y - 25), FONT_HERSHEY_SIMPLEX, 2.0, Scalar(0, 255, 255), 4);
        putText(result_image, to_string(re_number), Point2f(num_points[i].pts[0].x - 5, num_points[i].pts[0].y - 5), FONT_HERSHEY_SIMPLEX, 2.0, Scalar(0, 255, 0), 4);
        armor_red_list[i].num = re_number;

    }
  }
}

//---------动态识别相关函数-----------//
void TestNode::gettargets(Armor &armor){
  TrackedTarget new_target;
  new_target.center =Point2f(0.0f,0.0f);
  for (int i = 0; i < 4; i++)
  {
    new_target.center += armor.pts[i];
  }
  new_target.center /= 4.0;
  new_target.size = armor.area;
  new_target.id=-1;
  new_target.miss_count=0;
  new_target.type="armor";
  current_frame_targets.push_back(new_target);
}

void TestNode::framematch(){
  for(auto &track:tracker_targets){
    track.miss_count++;
  }
  
  for (auto &current_track : current_frame_targets)
  {
    float min_dist = 10000.0f;
    int match_id = -1;//匹配到的上一帧的对应id
    for (auto &track : tracker_targets)
    {
      float dist = norm(current_track.center - track.center);
      float size_ratio = min(current_track.size, track.size) / max(current_track.size, track.size);
      if (track.type != current_track.type)
        continue;
      if(dist>50.0f)
        continue;
      if(size_ratio<0.7f)
        continue;
      match_id = track.id;
      if(dist<min_dist){
        min_dist = dist;
        match_id = track.id;
      }
    }
      //匹配成功
    if(match_id!=-1){
      current_track.id = match_id;
      for(auto&track:tracker_targets){
        if(track.id==match_id){
          track.size = current_track.size;
          track.center = current_track.center;
          track.miss_count = 0;
          
        }
      }
    }
    //匹配失败，创建新目标，给id
    else{
      current_track.id = next_target_id++;
      tracker_targets.push_back({current_track.id,
                                 current_track.center,
                                 current_track.size,
                                 current_track.type,
                                 0});
    }
    //删去太久每匹配到的目标
    vector<TrackedTarget> new_tracker_targets;
    for(auto&track:tracker_targets){
      if(track.miss_count<=MAX_MISS_FRAMES){
        new_tracker_targets.push_back(track);
      }
    }
    tracker_targets = new_tracker_targets;
  }
}
//---------消息发布函数-----------//
void TestNode::publishmsg(const sensor_msgs::msg::Image::SharedPtr &msg){
  referee_pkg::msg::MultiObject msg_object;
        msg_object.header = msg->header;
        msg_object.num_objects = armor_red_list.size()+rect_list.size()+sphere_list.size();
        vector<string> types = {"armor_red_","rect","sphere"};
        for (int k = 0; k < armor_red_list.size(); k++)
        {
          referee_pkg::msg::Object obj;
          obj.target_type = types[0]+to_string(armor_red_list[k].num);
          Armor p = armor_red_list[k];
          geometry_msgs::msg::Point new_corner;
          for (int i = 0; i < 4;i++){
            new_corner.x=p.pts[i].x;
            new_corner.y = p.pts[i].y;
            new_corner.z = 0.0;
            obj.corners.push_back(new_corner);
          }
          msg_object.objects.push_back(obj);
        }
        //
        for (int k = 0; k < rect_list.size();k++){
          referee_pkg::msg::Object obj;
          obj.target_type = types[1];
          Rects p = rect_list[k];
          geometry_msgs::msg::Point new_corner;
          for (int i = 0; i < 4;i++){
            new_corner.x=p.pts[i].x;
            new_corner.y = p.pts[i].y;
            new_corner.z = 0.0;
            obj.corners.push_back(new_corner);
          }
          msg_object.objects.push_back(obj);
        }
        //
         for (int k = 0; k < sphere_list.size();k++){
          referee_pkg::msg::Object obj;
          obj.target_type = types[2];
          Sphere p = sphere_list[k];
           geometry_msgs::msg::Point new_corner;
          for (int i = 0; i < 4;i++){
            new_corner.x=p.pts[i].x;
            new_corner.y = p.pts[i].y;
            new_corner.z = 0.0;
            obj.corners.push_back(new_corner);
          }
          msg_object.objects.push_back(obj);
        }
          Target_pub->publish(msg_object);
}
//---------主函数-----------//
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = make_shared<TestNode>("TestNode");
  RCLCPP_INFO(node->get_logger(), "Starting TestNode");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}