//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        float a,b;
        float u_img = u * width;
        float u_img_floor = floor(u_img);
        a = u_img - u_img_floor;

        float v_img = (1 - v) * height;
        float v_img_floor = floor(v_img);
        b = v_img - v_img_floor;

        auto color_u_top = (1-a)*image_data.at<cv::Vec3b>(int(v_img+1), int(u_img)) + a*image_data.at<cv::Vec3b>(int(v_img+1), int(u_img+1));
        auto color_u_down = (1-a)*image_data.at<cv::Vec3b>(int(v_img), int(u_img)) + a*image_data.at<cv::Vec3b>(int(v_img), int(u_img+1));
        
        auto color_v = (1-b)*color_u_down + b*color_u_top;
        auto color = color_v;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }


};
#endif //RASTERIZER_TEXTURE_H
