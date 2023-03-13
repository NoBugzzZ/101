#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
                  << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window)
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                     3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    // TODO: Implement de Casteljau's algorithm
    int len = control_points.size();
    if (len == 1)
    {
        return control_points.back();
    }
    std::vector<cv::Point2f> points;
    for (int i = 1; i < len; i++)
    {
        cv::Point2f point = (1 - t) * control_points[i - 1] + t * control_points[i];
        points.push_back(point);
    }
    return recursive_bezier(points, t);
}

// void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
// {
//     // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
//     // recursive Bezier algorithm.
//     for (float t = 0; t <= 1; t += 0.0001)
//     {
//         auto point = recursive_bezier(control_points, t);
//         window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
//         const float x[4] = {0, 0, 0.5, -0.5};
//         const float y[4] = {+0.5, -0.5, 0, 0};
//         int xnow = round(point.x), ynow = round(point.y);
//         float d = std::sqrt(std::pow(point.x - xnow, 2) + std::pow(point.y - ynow, 2));
//         for (int i = 0; i < 4; i++)
//         {
//             float x_neibor = floor(point.x + x[i]);
//             float y_neibor = floor(point.y + y[i]);
//             if (x_neibor >= 0 && x_neibor < 700 && y_neibor >= 0 && y_neibor < 700)
//             {
//                 float w = d / std::sqrt((std::pow(x_neibor - point.x, 2) + std::pow(y_neibor - point.y, 2)));
//                 // float w = 1/std::sqrt((std::pow(x_neibor-point.x,2)+std::pow(y_neibor-point.y,2)));
//                 window.at<cv::Vec3b>(y_neibor, x_neibor)[1] = std::max(float(window.at<cv::Vec3b>(y_neibor, x_neibor)[1]), 255 * w);
//             }
//         }
//     }
// }

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    // recursive Bezier algorithm.
    for (float t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        int bound = 1;
        int x = round(point.x);
        int y = round(point.y);
        float neibor_x[] = {-0.5, 0.5, -0.5, 0.5};
        float neibor_y[] = {0.5, 0.5, -0.5, -0.5};
        float d[]={0,0,0,0};

        float min=10;
        for(int i=0;i<4;i++){
            float toP_x=neibor_x[i]+x-point.x;
            float toP_y=neibor_y[i]+y-point.y;
            float l=sqrt(pow(toP_x,2)+pow(toP_y,2));
            d[i]=l;
            if(min>l){
                min=l;
            }
        }
        for (int i = 0; i < 4; i++)
        {
            float toP_x=neibor_x[i]+x-point.x;
            float toP_y=neibor_y[i]+y-point.y;
            if(neibor_x[i]+x>=0&&neibor_x[i]+x<=window.cols&&neibor_y[i]+y>=0&&neibor_y[i]+y<=window.rows){
                float pre=window.at<cv::Vec3b>(neibor_y[i]+y, neibor_x[i]+x)[1];
                window.at<cv::Vec3b>(neibor_y[i]+y, neibor_x[i]+x)[1] = std::max(pre,255*(min/d[i]));
            }
        }
    }
}

int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27)
    {
        for (auto &point : control_points)
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4)
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
