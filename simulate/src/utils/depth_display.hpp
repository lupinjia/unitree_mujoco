#ifndef DEPTH_DISPLAY_HPP
#define DEPTH_DISPLAY_HPP

#include "mujoco/simulate.h"
#include <unitree/common/thread/recurrent_thread.hpp>
#include <opencv2/opencv.hpp>
#include "msg/DepthImage_.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"

namespace mj = ::mujoco;
using namespace unitree::robot;
#define TOPIC_DEPTHIMAGE "rt/depthimage"

class DepthDisplay
{
public:
    DepthDisplay(const mj::Simulate *sim, int width, 
                int height, int crop_left, float depth_dt_s,
                int domain_id, const std::string& interface)
        : mj_sim_(sim), width_(width), height_(height), 
        crop_left_(crop_left)
    {
        depth_dt_us_ = static_cast<int>(depth_dt_s * 1e6);
        ChannelFactory::Instance()->Init(domain_id, interface);
        initDepthImagePublisherDDS();
        printf("DepthDisplay: initialized depth image publisher on topic %s\n", TOPIC_DEPTHIMAGE);
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "DepthDisplay", 
            UT_CPU_ID_NONE, depth_dt_us_, 
            std::bind(&DepthDisplay::displayDepthImage, this));
    }

    void initDepthImagePublisherDDS()
    {
        depth_image_msg_.width() = width_ - crop_left_;
        depth_image_msg_.height() = height_;
        depth_image_msg_.normalized_value().resize((width_ - crop_left_) * height_);
        depth_image_msg_.normalized_value().clear();
        for(int i = 0; i < (width_ - crop_left_) * height_; ++i)
        {
            depth_image_msg_.normalized_value().push_back(0.0f);
        }
        depth_image_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::DepthImage_>(TOPIC_DEPTHIMAGE));
        depth_image_publisher->InitChannel();
    }

    void displayDepthImage()
    {
        // Assuming depth image is rendered to a framebuffer or texture
        // 32 bit float channel 1
        cv::Mat depth_image(height_, width_ - crop_left_, CV_32FC1);

        // Fill depth_image with simulated depth data
        if (mj_sim_->depth_value_ == nullptr)
        {
            // fill with zeros if depth_value_ is not initialized
            depth_image.setTo(cv::Scalar(0));
        }
        else
        {
            for (int i = 0; i < height_; ++i)
            {
                for (int j = 0; j < width_ - crop_left_; ++j)
                {
                    depth_image.at<float>(i, j) = mj_sim_->depth_value_[i * (width_ - crop_left_) + j];
                }
            }
        }
        // flip around y axis
        cv::flip(depth_image, depth_image, 0);
        // fill depth_image into depth_image_msg_
        for (int i = 0; i < height_; ++i)
        {
            for (int j = 0; j < width_ - crop_left_; ++j)
            {
                depth_image_msg_.normalized_value()[i * (width_ - crop_left_) + j] = depth_image.at<float>(i, j);
            }
        }
        // publish depth image message
        if (depth_image_publisher != nullptr)
        {
            depth_image_publisher->Write(depth_image_msg_);
        }
        // Convert to 8-bit for display
        cv::Mat depth_image_8u;
        depth_image.convertTo(depth_image_8u, CV_8UC1, 255.0);

        // Display the depth image using OpenCV
        cv::imshow("Depth Image", depth_image_8u);
        cv::waitKey(1); // Wait for 1 ms to allow image to be displayed
    }

private:
    const mj::Simulate* mj_sim_;
    int width_;
    int height_;
    int crop_left_;
    int depth_dt_us_; // depth display interval in microseconds
    unitree::common::RecurrentThreadPtr thread_;
    ChannelPublisherPtr<unitree_go::msg::dds_::DepthImage_> depth_image_publisher;
    unitree_go::msg::dds_::DepthImage_ depth_image_msg_;
};

#endif // DEPTH_DISPLAY_HPP