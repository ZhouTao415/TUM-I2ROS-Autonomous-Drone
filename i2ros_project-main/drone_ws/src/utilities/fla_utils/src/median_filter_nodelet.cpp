// median_filter_nodelet.cpp

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <limits>
#include <ctime>
#include <memory>

#include "median_heap.h"

//#define DBG_TIMING

namespace fla_utils
{


class MedianFilterNodelet : public nodelet::Nodelet
{
public:
  MedianFilterNodelet()
  : aperture_size_(3)
  , stride_(1)
  , ignore_val_(-1)
  , queue_size_(5)
  {}

  virtual void onInit();

protected:
  int aperture_size_;
  int stride_;
  int ignore_val_;
  int queue_size_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_img_;
  image_transport::CameraPublisher pub_img_;

  void img_callback(const sensor_msgs::Image::ConstPtr& msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info);

  template <typename T>
    void replace_pixels(const cv::Mat& src, cv::Mat& dst, T src_val, T dst_val) const;

  template <typename T>
    void median_filt_with_ignore(const cv::Mat& src, cv::Mat& dst, const int k, const int ignore_val, const int stride=1) const;
};


template <typename T>
  void MedianFilterNodelet::replace_pixels(const cv::Mat& src, cv::Mat& dst, T src_val, T dst_val) const
{
  dst.create( src.size(), src.type() );

  // We need to replace all values matching ignore_val with max value
  cv::MatConstIterator_<T> p = src.begin<T>();
  cv::MatConstIterator_<T> p_end = src.end<T>();
  T* q = dst.ptr<T>(0);  // we know dst is contiguous
  for (; p != p_end; ++p, ++q) {
    if ( *p == src_val ) {
      *q = dst_val;
    } else {
      *q = *p;
    }
  }
}

template <typename T>
  void MedianFilterNodelet::median_filt_with_ignore(const cv::Mat& src, cv::Mat& dst, const int k, const int ignore_val, const int stride) const
{
  const int H = std::ceil(src.rows/static_cast<float>(stride));
  const int W = std::ceil(src.cols/static_cast<float>(stride));
  const int r = k / 2;

  dst.create( H, W, src.type() );

  T ign = static_cast<T>(ignore_val);

  MedianHeap<int> med_heap;

  // Now for each column
  for (int w=0, i=0; w < W; ++w, i+=stride) {
    // Calculate valid horizontal size of aperture
    const int r_min = -std::min(r, i);
    const int r_max = std::min(r, (src.cols-1)-i);
    const int min_good = k*k / 2;  // minimum number of good pixels in the window (ie if ignore val is in fact the most common, we'll assign ignore_val)
                                  // NOTE: incorrect at edges!

    const T* p = src.ptr<T>(0) + i;  // pointer to aperture center
    T* q = dst.ptr<T>(0) + w;  // pointer to output pixel
    // Init heap within starting box
    med_heap.clear();
    for (int x=r_min; x <=r_max; ++x) {
      for (int y=0; y <= r; ++y) {
        int f = p[x + y*src.cols];
        if ( f != ign ) {
          med_heap.push( f );
        }
      }
    }

    // Move aperture down column
    for (int h=0, j=0; h < H; ++h, j+=stride) {
      // Get current median if available
      if ( med_heap.size() <= min_good ) {
        *q = ign;
      } else {
        *q = med_heap.get_median();
      }

      // Add new bottom rows if they exist
      for (int k=r+1; k<=r+stride && k<src.rows-j; ++k) {
        for (int x=r_min; x<=r_max; ++x) {
          int f = p[x + k*src.cols];
          if ( f != ign ) {
            med_heap.push( f );
          }
        }
      }

      // Move aperture down one row
      p += stride*src.cols;
      q += W;

      // Remove top rows if they exist
      for (int k=r+1; k<=r+stride && k<=j+stride; ++k) {
        for (int x=r_min; x<=r_max; ++x) {
          int f = p[x - k*src.cols];
          if ( f != ign ) {
            med_heap.pop( f );
          }
        }
      }
    }
  }
}

void MedianFilterNodelet::onInit()
{
  pnh_ = getPrivateNodeHandle();
  nh_ = getNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh_));

  int aperture = aperture_size_;
  pnh_.getParam("aperture_size", aperture);
  if ( aperture >= 1 && aperture % 2 == 1 ) {
    aperture_size_ = aperture;
  } else {
    NODELET_WARN("[MedianFilterNodelet] Invalid aperture size specified: %d. Using default of %d.", aperture, aperture_size_);
  }
  int stride = stride_;
  pnh_.getParam("stride", stride);
  if ( stride >= 1 ) {
    stride_ = stride;
  } else {
    NODELET_WARN("[MedianFilterNodelet] Invalid stride specified: %d. Using default of %d.", stride, stride_);
  }

  pnh_.getParam("ignore_val", ignore_val_);

  int queue = queue_size_;
  pnh_.getParam("queue_size", queue);
  if ( queue >= 0 ) {
    queue_size_ = queue;
  } else {
    NODELET_WARN("[MedianFilterNodelet] Invalid queue size specified: %d. Using default of %d.", queue, queue_size_);
  }

  sub_img_ = it_->subscribeCamera("image_raw", queue_size_, &MedianFilterNodelet::img_callback, this);
  pub_img_ = it_->advertiseCamera("image_filt", queue_size_);
}

void MedianFilterNodelet::img_callback(const sensor_msgs::Image::ConstPtr& msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
{
  if ( pub_img_.getNumSubscribers() == 0 ) {
    return;
  }

  // Get CV pointer
  cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvShare(msg);

  // Check for bad input
  if ( cv_img->image.channels() > 4 || cv_img->image.channels() == 2 ) {
    NODELET_ERROR_STREAM("[MedianFilterNodelet] Image has unsupported number of channels: " << cv_img->image.channels() << std::endl);
    return;
  } else if (!(cv_img->image.depth() == CV_8U || cv_img->image.depth() == CV_16U) && cv_img->image.channels() > 1) {
    NODELET_ERROR_STREAM("[MedianFilterNodelet] Image has incompatible type." << std::endl);
    return;
  }

  cv::Mat buf;
  if ( cv_img->image.isContinuous() ) {
    buf = cv_img->image;
  } else {
    cv_img->image.copyTo(buf);
  }

  // Do median blurring
  cv_bridge::CvImagePtr cv_dst( new cv_bridge::CvImage() );

  #ifdef DBG_TIMING
    std::clock_t t_start = std::clock();
  #endif

  if ( ignore_val_ >= 0 ) {
    if ( cv_img->image.depth() == CV_8U ) {
      median_filt_with_ignore<uint8_t>(buf, cv_dst->image, aperture_size_, ignore_val_, stride_);
    } else if ( cv_img->image.depth() == CV_16U ) {
      median_filt_with_ignore<uint16_t>(buf, cv_dst->image, aperture_size_, ignore_val_, stride_);
    } else {
      NODELET_ERROR_STREAM("[MedianFilterNodelet] Image has depth which is incompatible with ignore value parameter." << std::endl);
      return;
    }
  } else {
    cv::medianBlur(buf, cv_dst->image, aperture_size_);
  }

  #ifdef DBG_TIMING
    std::clock_t t_end = std::clock();
    std::cout << "MedianBlur compute ms: " << static_cast<double>(t_end-t_start) / (1e-3*CLOCKS_PER_SEC) << std::endl;
  #endif

  sensor_msgs::CameraInfo cam_info_out(*cam_info);
  cam_info_out.binning_x = stride_;
  cam_info_out.binning_y = stride_;
  cv_dst->header = cv_img->header;
  cv_dst->encoding = cv_img->encoding;
  pub_img_.publish( *cv_dst->toImageMsg(), cam_info_out );
}

PLUGINLIB_EXPORT_CLASS(fla_utils::MedianFilterNodelet, nodelet::Nodelet);

}  // namespace fla_utils
