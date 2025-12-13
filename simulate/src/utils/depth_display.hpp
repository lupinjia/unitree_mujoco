#ifndef DEPTH_DISPLAY_HPP
#define DEPTH_DISPLAY_HPP

#include "mujoco/simulate.h"
#include <unitree/common/thread/recurrent_thread.hpp>
#include <opencv2/opencv.hpp>

namespace mj = ::mujoco;

class DepthDisplay
{
public:
    DepthDisplay(const mj::Simulate *sim, int width, 
                int height, int crop_left, float far_clip,
                float near_clip, float depth_dt_s)
        : mj_sim_(sim), width_(width), height_(height), 
        crop_left_(crop_left), far_clip_(far_clip), near_clip_(near_clip)
    {
        depth_dt_us_ = static_cast<int>(depth_dt_s * 1e6);
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "DepthDisplay", 
            UT_CPU_ID_NONE, depth_dt_us_, 
            std::bind(&DepthDisplay::displayDepthImage, this));
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

        // clip the depth values based on near and far clip planes
        for (int i = 0; i < depth_image.rows; ++i)
        {
            for (int j = 0; j < depth_image.cols; ++j)
            {
                float& depth = depth_image.at<float>(i, j);
                if (depth < near_clip_)
                {
                    depth = near_clip_;
                }
                if (depth > far_clip_)
                {
                    depth = far_clip_;
                }
            }
        }
        // normalize the depth values to [0, 1] based on far and near clip
        for (int i = 0; i < depth_image.rows; ++i)
        {
            for (int j = 0; j < depth_image.cols; ++j)
            {
                float& depth = depth_image.at<float>(i, j);
                depth = (depth - near_clip_) / (far_clip_ - near_clip_);
            }
        }
        // Convert to 8-bit for display
        cv::Mat depth_image_8u;
        depth_image.convertTo(depth_image_8u, CV_8UC1, 255.0);
        // flip around y axis
        cv::flip(depth_image_8u, depth_image_8u, 0);

        // Display the depth image using OpenCV
        cv::imshow("Depth Image", depth_image_8u);
        cv::waitKey(1); // Wait for 1 ms to allow image to be displayed
    }

private:
    const mj::Simulate* mj_sim_;
    int width_;
    int height_;
    int crop_left_;
    float far_clip_;
    float near_clip_;
    int depth_dt_us_; // depth display interval in microseconds
    unitree::common::RecurrentThreadPtr thread_;
};

#endif // DEPTH_DISPLAY_HPP