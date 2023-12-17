// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    auto v0 = _v[0]-_v[1],v1=_v[1]-_v[2],v2=_v[2]-_v[0];
    Eigen::Vector3f p=Eigen::Vector3f(x,y,0);
    auto p0 = p-_v[1],p1=p-_v[2],p2=p-_v[0];
    auto res_0 = v0.cross(p0),res_1 = v1.cross(p1),res_2 = v2.cross(p2);
    if (res_0.dot(res_1)>0 and res_1.dot(res_2)>0 and res_2.dot(res_0)>0){
            return true;
        }
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)//per tranangle
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),//first vertex of this trangle
                mvp * to_vec4(buf[i[1]], 1.0f),//next
                mvp * to_vec4(buf[i[2]], 1.0f)//next
        };
        //Homogeneous division
        for (auto& vec : v) {//per vertex
            vec /= vec.w();//perspective division
        }
        //Viewport transformation
        for (auto & vert : v)//per vertex
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)//per vertex of this trangle
        {
            t.setVertex(i, v[i].head<3>());//first 3 data of this vertex
            //t.setVertex(i, v[i].head<3>());//why?i think it's mistake
            //t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];//first vertex's color from color buffer
        auto col_y = col[i[1]];//next vertex
        auto col_z = col[i[2]];//next vertex

        t.setColor(0, col_x[0], col_x[1], col_x[2]);//set first vertex's color
        t.setColor(1, col_y[0], col_y[1], col_y[2]);//next
        t.setColor(2, col_z[0], col_z[1], col_z[2]);//next

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();//{v1,v2,v3}
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    float xmin=v[0].x(),xmax=v[0].x(),ymin=v[0].y(),ymax=v[0].y();
    
    for (auto& vert : v ) {
        if (vert[0]<xmin){xmin = vert[0];}
        if (vert[0]>xmax){xmax = vert[0];}
        if (vert[1]<ymin){ymin = vert[1];}
        if (vert[1]>ymax){ymax = vert[1];}
    }
    

    for (int y=ymin;y<=ymax;y++){
        if(y<0 or y>=height){continue;}
        for (int x=xmin;x<=xmax;x++){
            if(x<0 or x>=width){continue;}
            // If so, use the following code to get the interpolated z value.
            auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
            //std::cout<<color_res;
            if(insideTriangle(x+.5,y+.5,&t.v[0]) and z_interpolated<depth_buf[get_index(int(x),int(y))]){
                Eigen::Vector3f point = Eigen::Vector3f(x,y,z_interpolated);
                depth_buf[get_index(int(x),int(y))]=z_interpolated;
                Eigen::Vector3f color_res = t.color[0] * alpha * 255 + t.color[1] * beta * 255 + t.color[2] * gamma * 255;
                set_pixel(point,color_res);
            }
        }
    }
    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (point.y())*width + point.x();
    //auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on