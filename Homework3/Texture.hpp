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
        if (u < 0)
            u = 0;
        if (u > 1)
            u = 1;
        if (v < 0)
            v = 0;
        if (v > 1)
            v = 1;
        
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        int x0 = u_img, x1 = u_img + 1;
        int y0 = v_img, y1 = v_img + 1;

        float s = u_img-x0;
        float t = v_img-y0;
        
        auto getf = [&](cv::Vec3b color){
            return Eigen::Vector3f(color[0], color[1], color[2]);
        };

        auto u00 = getf(image_data.at<cv::Vec3b>(x0, y0));
        auto u01 = getf(image_data.at<cv::Vec3b>(x0, y1));
        auto u10 = getf(image_data.at<cv::Vec3b>(x1, y0));
        auto u11 = getf(image_data.at<cv::Vec3b>(x1, y1));

        auto lerp = [&](float x, Eigen::Vector3f v0, Eigen::Vector3f v1){
            return v0 + x * (v1 - v0);
        };

        auto u0 = lerp(s, u00, u10);
        auto u1 = lerp(s, u01, u11);
        auto color = lerp(t, u0, u1);
        return color;
    }


};
#endif //RASTERIZER_TEXTURE_H
