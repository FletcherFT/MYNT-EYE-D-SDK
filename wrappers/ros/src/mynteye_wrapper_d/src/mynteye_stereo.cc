#include <ros/ros.h>
// ROS Image specific includes
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>

#include "mynteyed/camera.h"
#include "mynteyed/utils.h"
#include "mynteyed/util/rate.h"


MYNTEYE_USE_NAMESPACE

namespace {

void matrix_3x1(const double (*src1)[3], const double (*src2)[1],
    double (*dst)[1]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 1; j++) {
      for (int k = 0; k < 3; k++) {
        dst[i][j] += src1[i][k] * src2[k][j];
      }
    }
  }
}

void matrix_3x3(const double (*src1)[3], const double (*src2)[3],
    double (*dst)[3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        dst[i][j] += src1[i][k] * src2[k][j];
      }
    }
  }
}

}  // namespace

namespace enc = sensor_msgs::image_encodings;



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "mynteye_d1200_node");
  ros::NodeHandle nh("mynteye");
  ros::NodeHandle leftnh("mynteye/left");
  ros::NodeHandle rightnh("mynteye/right");

  // Image publishers
  image_transport::ImageTransport it(nh);

  image_transport::CameraPublisher pub_left = it.advertiseCamera("left/image_raw", 1);
  image_transport::CameraPublisher pub_right = it.advertiseCamera("right/image_raw", 1);
  int fps = 5;
  nh.getParamCached("fps", fps);

  const std::string cname="mynteye_d1200";
  const std::string url="package://mynteye_wrapper_d/configurations/${NAME}.yaml";
  camera_info_manager::CameraInfoManager cinfo(nh,cname,url);

  camera_info_manager::CameraInfoManager cinfoleft_(leftnh, "left", url);
  camera_info_manager::CameraInfoManager cinforight_(rightnh, "right", url);


  // Initialize MYNTEYE camera
  Camera cam;
  DeviceInfo devInfo;
  if (!util::select(cam, &devInfo)) {
    return 1;
  }

  // Set parameters for D1200 camera
  OpenParams params(devInfo.index);
  // Framerate: 30(default), [0,60], [30](STREAM_2560x720)
  params.framerate = fps;
  // Device mode, default DEVICE_ALL
  //   DEVICE_COLOR: IMAGE_LEFT_COLOR ✓ IMAGE_RIGHT_COLOR ? IMAGE_DEPTH x
  //   DEVICE_DEPTH: IMAGE_LEFT_COLOR x IMAGE_RIGHT_COLOR x IMAGE_DEPTH ✓
  //   DEVICE_ALL:   IMAGE_LEFT_COLOR ✓ IMAGE_RIGHT_COLOR ? IMAGE_DEPTH ✓
  // Note: ✓: available, x: unavailable, ?: depends on #stream_mode
  params.dev_mode = DeviceMode::DEVICE_COLOR;
  // Color mode: raw(default), rectified
  params.color_mode = ColorMode::COLOR_RAW;
  // Stream mode: left+right color
  params.stream_mode = StreamMode::STREAM_2560x720;  // hd
  params.color_stream_format = StreamFormat::STREAM_MJPG;
  //params.depth_stream_format = StreamFormat::STREAM_YUYV;
  // Infrared intensity: 0(default), [0,10]
  params.ir_intensity = 0;
  // Auto-exposure: true(default), false
  // params.state_ae = false;

  // Auto-white balance: true(default), false
  // params.state_awb = false;

  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Opened " << devInfo.name << " device." << std::endl;

  Rate rate(params.framerate);
  
  std_msgs::Header header;
  std::string color_frame_id = "mynt_eye_d1200";
  header.frame_id = color_frame_id;


  // loop
  ros::Rate loop_rate(params.framerate);
  while (nh.ok()) {

    bool left_sub = pub_left.getNumSubscribers() > 0;
    bool right_sub = pub_right.getNumSubscribers() > 0;

    header.stamp = ros::Time().now();

    //if (left_sub){
      sensor_msgs::CameraInfoPtr infoleft(new sensor_msgs::CameraInfo(cinfoleft_.getCameraInfo()));
      auto left_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
      if (left_color.img){
        cv::Mat left = left_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
        sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(header, enc::BGR8, left).toImageMsg();
        infoleft->header.frame_id = left_msg->header.frame_id;
        infoleft->header.stamp = left_msg->header.stamp;
        pub_left.publish(left_msg, infoleft);
      }
    //}
    //if (right_sub){
      sensor_msgs::CameraInfoPtr inforight(new sensor_msgs::CameraInfo(cinforight_.getCameraInfo()));
      auto right_color = cam.GetStreamData(ImageType::IMAGE_RIGHT_COLOR);
      if(right_color.img){
        cv::Mat right = right_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
        sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(header, enc::BGR8, right).toImageMsg();
        inforight->header.frame_id = right_msg->header.frame_id;
        inforight->header.stamp = right_msg->header.stamp;
        pub_right.publish(right_msg, inforight);
      }
    //}
    
    loop_rate.sleep();
  }

  cam.Close();
  return 0;
}