#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f Rotation_matrix(Eigen::Vector3f v, double angle) {
    double x = v[0];
    double y = v[1];
    double z = v[2];

    double sin_zetta = std::sin(angle);
    double cos_zetta = std::cos(angle);
    double one_sub_cos_zetta = 1 - cos_zetta;
    Eigen::Matrix4f translate;
    translate << cos_zetta + one_sub_cos_zetta * x * x, one_sub_cos_zetta * x * y - sin_zetta * z, one_sub_cos_zetta * x * z + sin_zetta * y,0,
    one_sub_cos_zetta * x * y + sin_zetta * z, cos_zetta + one_sub_cos_zetta * y * y, one_sub_cos_zetta * y * z - sin_zetta * x,0,
    one_sub_cos_zetta * x * z - sin_zetta * y, one_sub_cos_zetta * y * z + sin_zetta * x, cos_zetta + one_sub_cos_zetta * z * z,0,
    0,0,0,1;
    return translate;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
     0, 1, 0, -eye_pos[1],
      0, 0, 1,-eye_pos[2],
       0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    
    Eigen::Matrix4f translate;
    // translate << std::cos(rotation_angle),-std::sin(rotation_angle),0,0,
    // std::sin(rotation_angle),std::cos(rotation_angle),0,0,
    // 0,0,1,0,
    // 0,0,0,1;

    translate = Rotation_matrix(Eigen::Vector3f(std::sqrt(3)/3,std::sqrt(3)/3,std::sqrt(3)/3),rotation_angle);


    model = get_view_matrix({0, 0, 5})*translate*get_view_matrix({0, 0, -5})*model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float t,b,l,r;
    t = zNear * std::tan(eye_fov/2);
    b = -t;
    r = t * aspect_ratio;
    l = -r;
    Eigen::Matrix4f translate;
    translate << 2*zNear/(r-l),0,(r+l)/(r-l),0,
    0,2*zNear/(t-b),(b+t)/(b-t),0,
    0,0,(zFar+zNear)/(zNear-zFar),2*zFar*zNear/(zFar-zNear),
    0,0,1,0;
    projection = translate*projection;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2},{4, 0, -4}, {0, 4, -4}, {-4, 0, -4}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;
    float FOV=45;
    float aspect_ratio = 1;
    float zNear = 0.1;
    float zFar = 50;
    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(FOV/180*MY_PI, aspect_ratio, zNear, zFar));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(FOV/180*MY_PI, aspect_ratio, zNear, zFar));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 5./180*MY_PI;
        }
        if (key == 'd') {
            angle -= 5./180*MY_PI;
        }
        if(key == 'i'){
            FOV +=4.;
        }
        if(key == 'u'){
            FOV -=4.;
        }
    }

    return 0;
}
