//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    pdf = emit_area_sum;//I add
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection res;
    res = intersect(ray);
    if (!res.happened){return Vector3f(0);}
    Vector3f Kd_chache =res.m->Kd;
    float RR = std::max(std::max(Kd_chache.x,Kd_chache.y),Kd_chache.z);
    float RR_rand = get_random_float();
    float RR_fix = 1;
    if (RR>RR_rand and depth>2){RR_fix = 1/RR;return Vector3f(0);}
    if (res.m->hasEmission() and !depth){return res.m->getEmission();}
    if (res.m->hasEmission()){return Vector3f(0);}
    Vector3f N = normalize(res.normal);
    if(dotProduct(N,ray.direction)>0){
        N=-N;
    }
    Vector3f wi = ray.direction;
    Vector3f wo = normalize(res.m->sample(wi,N));
    Intersection light_pos;
    float pdf;
    sampleLight(light_pos,pdf);
    Vector3f origin2light = light_pos.coords-res.coords;
    Vector3f origin2light_nor = origin2light;
    origin2light_nor=normalize(origin2light_nor);
    Vector3f L_N = normalize(light_pos.normal);
    float cos_zeta=dotProduct(L_N,origin2light_nor);
    cos_zeta = cos_zeta>0?cos_zeta:-cos_zeta;
    Vector3f E;
    if(intersect(Ray(res.coords,origin2light_nor)).distance<origin2light.norm()-(1e-3)){
        E = Vector3f(0.);
    }else{
        E = Vector3f(1.);
    }
    Vector3f lt_c=light_pos.m->getEmission()/dotProduct(origin2light,origin2light);
    Vector3f Fn_color = res.m->getEmission() + (castRay(Ray(res.coords,wo),depth+1)*dotProduct(wo,N)/res.m->pdf(wi,wo,N) + E*lt_c*dotProduct(N,origin2light_nor)*cos_zeta/pdf) * res.m->eval(wi,wo,N);


    return Fn_color*RR_fix;
}