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
    auto p = intersect(ray);
    Vector3f hit_color(0);
    const float epsilon = 0.0001f;

    
    if (p.happened)
    {
        if (p.m->hasEmission()){
            return p.m->getEmission();
        }
        
        Vector3f wo = (-ray.direction).normalized();
        
        // Contribution from the light source.
        Vector3f L_dir(0);
        {
            float pdf_light;
            Intersection x;
            sampleLight(x, pdf_light);
            Vector3f p2x = x.coords - p.coords;
            Vector3f ws = p2x.normalized();

            // If the ray is not blocked in the middle.
            auto t = intersect(Ray(p.coords, ws));
            if(((t.coords - x.coords)).norm() < epsilon)
            {
                Vector3f f_r = p.m->eval(ws, wo, p.normal);
                float r = dotProduct(p2x, p2x);
                float cosA = std::max(.0f, dotProduct(ws, p.normal));
                float cosB = std::max(.0f, dotProduct(-ws, x.normal));
                L_dir = x.emit * f_r * cosA * cosB / r / pdf_light;
            }
        }

        // Contribution from other reflectors.
        Vector3f L_indir(0);
        {
            float P_RR = get_random_float();
            if(P_RR < RussianRoulette)
            {
                Vector3f wi = p.m->sample(wo, p.normal).normalized();
                float pdf = p.m->pdf(wi, wo, p.normal);
                Intersection q = intersect(Ray(p.coords, wi));
                if(q.happened && !q.m->hasEmission()) // if ray hit a non-emitting object at q
                {
                    Vector3f f_r = p.m->eval(wi, wo, p.normal);
                    float cos = std::max(.0f, dotProduct(wi, p.normal));
                    L_dir = castRay(Ray(p.coords, wi), depth) * f_r * cos / pdf / RussianRoulette;
                }
            }
        }
        hit_color = L_dir + L_indir;
    }

    return hit_color;
}