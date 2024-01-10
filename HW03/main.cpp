#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

// Eigen::Matrix4f Rotation_matrix(Eigen::Vector3f v, double angle) {//new
//     double x = v[0];
//     double y = v[1];
//     double z = v[2];

//     double sin_zetta = std::sin(angle);
//     double cos_zetta = std::cos(angle);
//     double one_sub_cos_zetta = 1 - cos_zetta;
//     Eigen::Matrix4f translate;
//     translate << cos_zetta + one_sub_cos_zetta * x * x, one_sub_cos_zetta * x * y - sin_zetta * z, one_sub_cos_zetta * x * z + sin_zetta * y,0,
//     one_sub_cos_zetta * x * y + sin_zetta * z, cos_zetta + one_sub_cos_zetta * y * y, one_sub_cos_zetta * y * z - sin_zetta * x,0,
//     one_sub_cos_zetta * x * z - sin_zetta * y, one_sub_cos_zetta * y * z + sin_zetta * x, cos_zetta + one_sub_cos_zetta * z * z,0,
//     0,0,0,1;
//     return translate;
// }



Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
    Eigen::Matrix4f projection;
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

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColor(payload.tex_coords[0],payload.tex_coords[1]);

    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    auto view_vec = eye_pos - point;

    for(auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        auto ref_vec = reflect(view_vec.normalized(),normal.normalized()).normalized();
        auto loght_vec = light.position-point;
        auto r = loght_vec.dot(loght_vec);
        auto cos = ref_vec.dot(loght_vec.normalized());//ref
        //cos = (view_vec.normalized()+loght_vec.normalized()).normalized().dot(normal);//haf
        auto lambort = view_vec.normalized().dot(normal.normalized());
        (lambort < 0)?0:lambort;
        (cos<0)?0:cos;
        result_color += ka.cwiseProduct(amb_light_intensity) + lambort*kd.cwiseProduct(light.intensity)/float(r) + light.intensity.cwiseProduct(ks)/float(r)*pow(cos,p);
    }
    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    auto view_vec = eye_pos - point;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        auto ref_vec = reflect(view_vec.normalized(),normal.normalized()).normalized();
        auto loght_vec = light.position-point;
        auto r = loght_vec.dot(loght_vec);
        auto cos = ref_vec.dot(loght_vec.normalized());//ref
        //cos = (view_vec.normalized()+loght_vec.normalized()).normalized().dot(normal);//haf
        auto lambort = view_vec.normalized().dot(normal.normalized());
        (lambort < 0)?0:lambort;
        (cos<0)?0:cos;
        result_color += ka.cwiseProduct(amb_light_intensity) + lambort*kd.cwiseProduct(light.intensity)/float(r) + light.intensity.cwiseProduct(ks)/float(r)*pow(cos,p);
    }

    return result_color * 255.f;
}


Eigen::Vector3f get_bump_testurecolor(float x,float y,const fragment_shader_payload& payload){
    Eigen::Vector3f res = payload.texture->getColor(x,y);
    return res;
}

Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};
    //fix the matrix right
    Eigen::Matrix3f Fix;
    Eigen::Vector3f vx = Eigen::Vector3f(-1,0,0);
    Eigen::Vector3f vy = Eigen::Vector3f(0,-1,0);
    Eigen::Vector3f vz = Eigen::Vector3f(0,0,-1);
    Fix << vx,vy,vz;
    //fixing
    //l1.position = Fix * l1.position;
    //l2.position = Fix * l2.position;


    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 5;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)
    float w = payload.texture->width;
    float h = payload.texture->height;

    Eigen::Vector3f n = normal.normalized();
    float x = n[0];
    float y = n[1];
    float z = n[2];
    Eigen::Vector3f t(x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z));
    Eigen::Vector3f b = normal.cross(t);
    //Eigen::Vector3f b = t.cross(n);
    Eigen::Matrix3f TBN;
    TBN<<t,b,n;
    //TBN<<t.x(),b.x(),n.x(),t.y(),b.y(),n.y(),t.z(),b.z(),n.z();
    //TBN<<t.x(),t.y(),t.z(),b.x(),b.y(),b.z(),n.x(),n.y(),n.z();
    float u = payload.tex_coords[0];
    float v = payload.tex_coords[1];
    float uu = u + 1.f/w;
    float vv = v + 1.f/h;

    float dhu,dhv;
    dhu = payload.texture->getColorBilinear(uu,v).norm()-payload.texture->getColorBilinear(u,v).norm();
    dhv = payload.texture->getColorBilinear(u,vv).norm()-payload.texture->getColorBilinear(u,v).norm();
    float dU = kh * kn * dhu;
    float dV = kh * kn * dhv;
    Eigen::Vector3f ln;
    ln<<-dU,-dV,1.0f;
    n = (TBN * ln).normalized();
    normal = (n.dot(payload.normal)<0)?n:-n;
    normal = n;
    point+=kn*kh*normal*(payload.texture->getColorBilinear(u,v).norm());
    Eigen::Vector3f view_vec = eye_pos - point;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f ref_vec = reflect(view_vec.normalized(),normal).normalized();
        Eigen::Vector3f loght_vec = light.position-point;
        float r = loght_vec.dot(loght_vec);
        float cos = ref_vec.dot(loght_vec.normalized());//ref
        cos = (cos>0)?cos:0;
        //cos = (view_vec.normalized()+loght_vec.normalized()).normalized().dot(normal);//haf
        float lambort = view_vec.normalized().dot(normal);
        lambort = (lambort>0)?lambort:0;
        Eigen::Vector3f la = ka.cwiseProduct(amb_light_intensity);
        Eigen::Vector3f ld = kd.cwiseProduct(light.intensity)/float(r)*lambort;
        Eigen::Vector3f ls = ks.cwiseProduct(light.intensity)/float(r)*pow(cos,p);
        result_color += la + ld + ls;
        //result_color += light.intensity.cwiseProduct(ks)/float(r)*pow(cos,p);
        //result_color += Eigen::Vector3f(lambort/2.f,lambort/2.f,lambort/2.f);
    }
    // auto test = view_vec.normalized().dot(normal);
    // test = (test>0)?test:-test;
    // return test * Eigen::Vector3f(255.f,255.f,255.f);
    //result_color= kd;
    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f return_color = {0, 0, 0};
    // if (payload.texture)
    // {
    //     // TODO: Get the texture value at the texture coordinates of the current fragment
    //     return_color = payload.texture->getColor(payload.tex_coords[0],payload.tex_coords[1]);
    // }
    float u = payload.tex_coords[0];
    float v = payload.tex_coords[1];

    return_color = get_bump_testurecolor(u,v,payload);

    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color =  payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)0
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)

    float w = payload.texture->width;
    float h = payload.texture->height;

    Eigen::Vector3f n = normal;
    float x = n[0];
    float y = n[1];
    float z = n[2];
    Eigen::Vector3f t(x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z));
    Eigen::Vector3f b = n.cross(t);
    Eigen::Matrix3f TBN;
    TBN<<t,b,n;
    
    float uu = u + 1./w;
    float vv = v + 1./h;

    float dhu,dhv;
    dhu = payload.texture->getColor(uu,v).norm()-payload.texture->getColor(u,v).norm();
    dhv = payload.texture->getColor(u,vv).norm()-payload.texture->getColor(u,v).norm();
    float dU = kh * kn * dhu;
    float dV = kh * kn * dhv;
    Eigen::Vector3f ln;
    ln<<-dU,-dV,1;
    n = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = n;

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{

    // Eigen::Vector3f v1(1,2,2);
    // Eigen::Vector3f v2(4,5,6);
    // Eigen::Vector3f v3(7,8,8);
    // Eigen::Matrix3f MTA;
    // MTA<<v1,v2,v3;
    // Eigen::Matrix3f MTA2;
    // MTA2<<1,2,3,4,5,6,7,8,9;
    // std::cout<<MTA<<MTA2<<std::endl;
    // return 0;

    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,5};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;


    float FOV=45;
    float aspect_ratio = 1;
    float zNear = 0.1;
    float zFar = 50;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(FOV/180*MY_PI, aspect_ratio, zNear, zFar));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(FOV/180*MY_PI, aspect_ratio, zNear, zFar));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
