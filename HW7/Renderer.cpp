//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"

#include <thread>
#include <mutex>

std::mutex mtx;

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);
    int all = scene.width * scene.height;

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    //int m = 0;

    int thread_num = 12;
    std::thread th[thread_num];


    std::vector<int> m(thread_num);

    float down=0;

    // change the spp value to change sample ammount
    float spp = 64;
    std::cout << "SPP: " << spp << "\n";
    auto rander_pixal = [&](int start_id){
        uint32_t i,j;
        for(m[start_id] = start_id;m[start_id]<all;m[start_id]+=thread_num){
                // generate primary ray direction
                i = m[start_id] % scene.width;
                j = m[start_id] / scene.width;
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                        imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                for (int k = 0; k < spp; k++){
                    if(spp-k<1){
                        if(get_random_float()<spp-k){
                            framebuffer[m[start_id]] += scene.castRay(Ray(eye_pos, dir), 0) / (spp-k);  
                            continue;
                        }
                        continue;
                    }
                    framebuffer[m[start_id]] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
                }
            if((j / (float)scene.height)>down){
                down = j / (float)scene.height;
                mtx.lock();
                UpdateProgress(down);
                mtx.unlock();
            }
        }
    };
    int start_ = 0;
    for(auto &per_th:th){
        per_th=std::thread(rander_pixal,start_++);
    }
    for(auto &per_th:th){
        per_th.join();
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
