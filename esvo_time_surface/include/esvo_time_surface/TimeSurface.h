#ifndef esvo_time_surface_H_
#define esvo_time_surface_H_

//ROS
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <cv_bridge/cv_bridge.h>
//cv_bridge用于ROS图像和OpenCV图像的转换
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//rpg msgs，引入message使用头文件
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <deque>
#include <mutex>
#include <Eigen/Eigen>

namespace esvo_time_surface
{
#define NUM_THREAD_TS 1
using EventQueue = std::deque<dvs_msgs::Event>;
// uint16 x，uint16 y，time ts，bool polarity；
// using = typedef
  
  
// EventQueueMat 类
// 变量：size_t width_，size_t height_，图大小 size_t queueLen_，
// std::vector<EventQueue> eqMat_    存储事件队列
// size_t的取值range是目标平台下最大可能的数组尺寸，方便移植
// 函数：
// insideImage，判断size_t x, size_t y 是否在图像范围内
// getEventQueue，注意按x方向存储
// insertEvent，注意队列最大长度queueLen_，超过pop_front
// getMostRecentEventBeforeT  用的ROS::time，T时间前最近的事件，赋给指针
class EventQueueMat 
{
public:
  EventQueueMat(int width, int height, int queueLen)
  {
    width_ = width;
    height_ = height;
    queueLen_ = queueLen;
    eqMat_ = std::vector<EventQueue>(width_ * height_, EventQueue());
  }

  void insertEvent(const dvs_msgs::Event& e)
  {
    if(!insideImage(e.x, e.y))
      return;
    else
    {
      EventQueue& eq = getEventQueue(e.x, e.y);
      eq.push_back(e);
      //用的while
      while(eq.size() > queueLen_)
        eq.pop_front();
    }
  }
  // 使用ROS::time
  bool getMostRecentEventBeforeT(
    const size_t x,
    const size_t y,
    const ros::Time& t,
    dvs_msgs::Event* ev)
  {
    if(!insideImage(x, y))
      return false;

    EventQueue& eq = getEventQueue(x, y);
    if(eq.empty())
      return false;
  //从队列尾开始遍历，也就是说插入事件不是根据时间
    for(auto it = eq.rbegin(); it != eq.rend(); ++it)
    {
      const dvs_msgs::Event& e = *it;
      if(e.ts < t)
      {
        *ev = *it;
        return true;
      }
    }
    return false;
  }

  void clear()
  {
    eqMat_.clear();
  }

  bool insideImage(const size_t x, const size_t y)
  {
    return !(x < 0 || x >= width_ || y < 0 || y >= height_);
  }
  //声明为inline
  inline EventQueue& getEventQueue(const size_t x, const size_t y)
  {
    return eqMat_[x + width_ * y];
  }

  size_t width_;
  size_t height_;
  size_t queueLen_;
  std::vector<EventQueue> eqMat_;
};

  

class TimeSurface
{
  struct Job
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EventQueueMat* pEventQueueMat_;
    cv::Mat* pTimeSurface_;
    size_t start_col_, end_col_;
    size_t start_row_, end_row_;
    size_t i_thread_;
    ros::Time external_sync_time_;
    double decay_sec_;
  };

public:
  TimeSurface(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  //  ros::NodeHandle nh;
  //  ros::NodeHandle nh_private("~");
  //  esvo_time_surface::TimeSurface ts(nh, nh_private);
  virtual ~TimeSurface();

private:
  ros::NodeHandle nh_;
  // core
  void init(int width, int height);
  void createTimeSurfaceAtTime(const ros::Time& external_sync_time);// single thread version (This is enough for DAVIS240C and DAVIS346)
  void createTimeSurfaceAtTime_hyperthread(const ros::Time& external_sync_time); // hyper thread version (This is for higher resolution)
  void thread(Job& job);

  // callbacks
  void syncCallback(const std_msgs::TimeConstPtr& msg);
  //Header header，uint32 height，uint32 width ，Event[] events

  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  // utils
  void clearEventQueue();

  // calibration parameters
  cv::Mat camera_matrix_, dist_coeffs_;
  cv::Mat rectification_matrix_, projection_matrix_;
  std::string distortion_model_;
  cv::Mat undistort_map1_, undistort_map2_;
  Eigen::Matrix2Xd precomputed_rectified_points_;

  // sub & pub
  ros::Subscriber event_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Subscriber sync_topic_;
  image_transport::Publisher time_surface_pub_;

  // online parameters
  bool bCamInfoAvailable_;
  bool bUse_Sim_Time_;  
  cv::Size sensor_size_;
  ros::Time sync_time_;
  bool bSensorInitialized_;

  // offline parameters
  double decay_ms_;
  bool ignore_polarity_;
  int median_blur_kernel_size_;
  int max_event_queue_length_;
  int events_maintained_size_;

  // containers
  EventQueue events_;
  std::shared_ptr<EventQueueMat> pEventQueueMat_;

  // thread mutex
  std::mutex data_mutex_;

  // Time Surface Mode
  // Backward: First Apply exp decay on the raw image plane, then get the value
  //           at each pixel in the rectified image plane by looking up the
  //           corresponding one (float coordinates) with bi-linear interpolation.
  // Forward: First warp the raw events to the rectified image plane, then
  //          apply the exp decay on the four neighbouring (involved) pixel coordinate.
  enum TimeSurfaceMode
  {
    BACKWARD,// used in the T-RO20 submission
    FORWARD
  } time_surface_mode_;
};
} // namespace esvo_time_surface
#endif // esvo_time_surface_H_
