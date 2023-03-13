//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture
{
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name)
    {
        image_data = cv::imread(name);
        cv::pyrDown(image_data,image_data);
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
        Eigen::Vector2f point = {u * width, (1 - v) * height};
        Eigen::Vector2f center={round(point.x()),round(point.y())};
        float x = center.x();
        float y = center.y();
        Eigen::Vector2f u00 = {x-0.5,y-0.5};
        Eigen::Vector2f u01 = {x+0.5,y-0.5};
        Eigen::Vector2f u10 = {x-0.5,y+0.5};
        Eigen::Vector2f u11 = {x+0.5,y+0.5};

        float s = point.x() - u00.x();
        float t = point.y() - u00.y();

        auto color_u00 = image_data.at<cv::Vec3b>(u00.y(), u00.x());
        auto color_u01 = image_data.at<cv::Vec3b>(u01.y(), u01.x());
        auto color_u0 = color_u00 + (color_u01 - color_u00) * s;

        auto color_u10 = image_data.at<cv::Vec3b>(u10.y(), u10.x());
        auto color_u11 = image_data.at<cv::Vec3b>(u11.y(), u11.x());
        auto color_u1 = color_u10 + (color_u11 - color_u10) * s;

        auto color = color_u0 + (color_u1 - color_u0) * t;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif // RASTERIZER_TEXTURE_H
