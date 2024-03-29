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

void AAdraw(cv::Point2f p,float r, cv::Mat &window,int rgb){
    for(int y = p.y-floor(r+1.);y<=p.y+floor(r+1.);y++){
        for(int x = p.x-floor(r+1.);x<=p.x+floor(r+1.);x++){
            cv::Point2f p_now = cv::Point2f(x,y);
            cv::Point2f vec_now = p_now-p;
            float r_now = pow(vec_now.x,2)+pow(vec_now.y,2);
            if(r_now>=pow(r,2)){continue;}
            r_now = sqrt(r_now);
            if(window.at<cv::Vec3b>(p_now)[rgb]<255*(1-r_now/r)){window.at<cv::Vec3b>(p_now)[rgb] = 255*(1-r_now/r);}
        }
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
        AAdraw(cv::Point2f(point.x, point.y),5,window,2);
        //window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f fun(std::vector<cv::Point2f> points, float t){
    if(points.size() == 1){
        return cv::Point2f(points[0]);
    }
    std::vector<cv::Point2f> chache_point;
    for(int i = 1;i<points.size();i++){
        cv::Point2f p0 = points[i-1];
        cv::Point2f p1 = points[i];
        float x = (1-t)*p0.x+t*p1.x;
        float y = (1-t)*p0.y+t*p1.y;
        chache_point.emplace_back(x, y);
    }
    return fun(chache_point,t);
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    std::vector<cv::Point2f> points = control_points;
    return fun(points,t);
    //return recursive_bezier(control_points,t);

}


void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.;t<=1.;t+=.001){
        AAdraw(recursive_bezier(control_points,t),5,window,1);
        //window.at<cv::Vec3b>(recursive_bezier(control_points,t))[1] = 255;
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

        //std::cout<<control_points.size()<<std::endl;
        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
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
