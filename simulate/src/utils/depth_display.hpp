#ifndef DEPTH_DISPLAY_HPP
#define DEPTH_DISPLAY_HPP

#include "mujoco/simulate.h"
#include <unitree/common/thread/recurrent_thread.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <mutex>
#include <vector>
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
        const int image_width = width_ - crop_left_;
        std::vector<float> frame(image_width * height_, 0.0f);
        {
            // Copy a complete frame while the renderer may be publishing the next one.
            const std::lock_guard<std::mutex> lock(mj_sim_->depth_value_mtx_);
            if (mj_sim_->depth_value_ != nullptr)
            {
                std::copy(mj_sim_->depth_value_, mj_sim_->depth_value_ + frame.size(), frame.begin());
            }
        }

        cv::Mat depth_image(height_, image_width, CV_32FC1);
        if (frame.empty())
        {
            depth_image.setTo(cv::Scalar(0));
        }
        else
        {
            std::copy(frame.begin(), frame.end(), depth_image.ptr<float>());
        }
        // flip around y axis
        cv::flip(depth_image, depth_image, 0);
        // fill depth_image into depth_image_msg_
        std::copy(depth_image.ptr<float>(), depth_image.ptr<float>() + frame.size(),
                  depth_image_msg_.normalized_value().begin());
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
