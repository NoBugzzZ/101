// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <array>

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

static std::tuple<bool,int> insideTriangleBySuperSampling(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f sampleList[4];
    sampleList[0]<<x+0.25,y+0.25,1;
    sampleList[1]<<x+0.75,y+0.25,1;
    sampleList[2]<<x+0.25,y+0.75,1;
    sampleList[3]<<x+0.75,y+0.75,1;
    for(int i=0;i<3;i++){
        Vector3f vec1 =_v[(i+1)%3]-_v[i];
        Vector3f vec2=_v[(i+2)%3]-_v[i];
        vec1[2]=0;
        vec2[2]=0;
        for(int j=0;j<4;j++){
            Vector3f vecToP=sampleList[j]-_v[i];
            vecToP[2]=0;
            if(sampleList[j][2]>0&&vec1.cross(vec2).dot(vec1.cross(vecToP))<0){
                sampleList[j][2]=0;
            } 
        }
    }
    int sum=0;
    for(int j=0;j<4;j++){
        sum+=sampleList[j][2];
    }
    if(sum>0){
        return {true,sum};
    }
    return {false,0};
}

static std::tuple<bool,std::vector<int>> insideTriangleBySuperSamplingWithoutBlackEdge(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f samplePoint[4];
    samplePoint[0]<<x+0.25,y+0.25,0;
    samplePoint[1]<<x+0.75,y+0.25,0;
    samplePoint[2]<<x+0.25,y+0.75,0;
    samplePoint[3]<<x+0.75,y+0.75,0;
    std::vector<int> inTriangle={1,1,1,1};

    for(int j=0;j<4;j++){
        for(int i=0;i<3;i++){
            Vector3f vec1 =_v[(i+1)%3]-_v[i];
            Vector3f vec2=_v[(i+2)%3]-_v[i];
            vec1[2]=0;
            vec2[2]=0;
            Vector3f vecToP=samplePoint[j]-_v[i];
            vecToP[2]=0;
            if(vec1.cross(vec2).dot(vec1.cross(vecToP))<0){
                inTriangle[j]=0;
                break;
            } 
        }
    }
    
    int sum=0;
    for(int j=0;j<4;j++){
        sum+=inTriangle[j];
    }
    if(sum>0){
        return {true,inTriangle};
    }
    return {false,inTriangle};
}

static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    for(int i=0;i<3;i++){
        Vector3f vec1 =_v[(i+1)%3]-_v[i];
        Vector3f vec2=_v[(i+2)%3]-_v[i];
        Vector3f vecToP=Vector3f(x+0.5,y+0.5,0)-_v[i];
        vec1[2]=0;
        vec2[2]=0;
        vecToP[2]=0;
        if(vec1.cross(vec2).dot(vec1.cross(vecToP))<0){
            return false;
        } 
    }
    return true;
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
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    int xmin=floor(std::min(std::min(v[0].x(),v[1].x()),v[2].x()));
    int xmax=ceil(std::max(std::max(v[0].x(),v[1].x()),v[2].x()));
    int ymin=floor(std::min(std::min(v[0].y(),v[1].y()),v[2].y()));
    int ymax=ceil(std::max(std::max(v[0].y(),v[1].y()),v[2].y()));

    for(int y=ymin;y<=ymax;y++){
        for(int x=xmin;x<=xmax;x++){
            // auto [flag,ssa]=insideTriangleBySuperSampling(x,y,t.v);
            // bool flag=insideTriangle(x,y,t.v);
            auto [flag,inTrianle]=insideTriangleBySuperSamplingWithoutBlackEdge(x,y,t.v);
            
            if(flag){
                // std::cout<<x<<" "<<y<<" "<<inTrianle[0]<<inTrianle[1]<<inTrianle[2]<<inTrianle[3]<<" "<<ssa<<std::endl;
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                // if(z_interpolated<depth_buf[get_index(x,y)]){
                //     depth_buf[get_index(x,y)]=z_interpolated;
                //     set_pixel(Vector3f(x,y,1),t.getColor());
                // }

                Vector3f color=t.getColor();
                Vector3f weighted_color;
                weighted_color<<0,0,0;
                for(int i=0;i<4;i++){
                    int index=get_index(x,y)*4+i;
                    if(inTrianle[i]==1){
                        if(z_interpolated<depth_buf_super_sample[index]){
                        depth_buf_super_sample[index]=z_interpolated;
                        frame_buf_super_sample[index]=color;
                    }
                    }
                    weighted_color+=frame_buf_super_sample[index];
                }
                set_pixel(Vector3f(x,y,1),weighted_color/4);
            }    
        }
    }
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

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
        std::fill(frame_buf_super_sample.begin(), frame_buf_super_sample.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buf_super_sample.begin(), depth_buf_super_sample.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_buf_super_sample.resize(w*h*4);
    depth_buf_super_sample.resize(w*h*4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on