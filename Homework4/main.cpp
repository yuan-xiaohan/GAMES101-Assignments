#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

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
    std::vector<cv::Point2f> points_list;

    if (control_points.size() == 1)
    {
        return control_points[0];
    }
    for(int i = 0; i < control_points.size()-1; i++)
    {
        auto &p_0 = control_points[i];
        auto &p_1 = control_points[i + 1];
        auto p = (1 - t) * p_0 + t * p_1;
        points_list.push_back(p);
    }
    cv::Point2f point = recursive_bezier(points_list, t);
    return point;
}

cv::Point2f anti_aliasing(cv::Point2f point, cv::Mat &window) 
{
    float x_r = round(point.x);
    float y_r = round(point.y);

    float d[4];
    cv::Point2f p_list[4] = {
        cv::Point2f(x_r + 0.5, y_r + 0.5), 
        cv::Point2f(x_r - 0.5, y_r + 0.5),
        cv::Point2f(x_r - 0.5, y_r - 0.5),
        cv::Point2f(x_r + 0.5, y_r - 0.5)};
    for(int i = 0; i < 4; i++){
        d[i] = sqrt(pow(point.x - p_list[i].x, 2) + pow(point.y - p_list[i].y, 2)) + 1e-8;
    }
    float dmin = std::min(std::min(std::min(d[0], d[1]), d[2]), d[3]);

    for (int i = 0; i < 4; i++)
    {
        float c = 255 * dmin / d[i];
        if (window.at<cv::Vec3b>(p_list[i].y, p_list[i].x)[1] < c)
        {
            window.at<cv::Vec3b>(p_list[i].y, p_list[i].x)[1] = c;
        } 
    }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(control_points, t);

        // anti_aliasing(point, window);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
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
