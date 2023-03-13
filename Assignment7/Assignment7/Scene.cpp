//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
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
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
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
    Intersection p = intersect(ray);
    // if (!p.happened)
    //     return {};
    if (p.obj != nullptr && p.obj->hasEmit())
    {
        return p.m->getEmission();
    }
    Vector3f L;

    if (p.happened)
    {
        // std::cout << "p " << p.happened << std::endl;
        Intersection lightSample;
        float pdf;
        sampleLight(lightSample, pdf);
        Vector3f temp = lightSample.coords - p.coords;
        Ray rayToLight(p.coords, normalize(temp), temp.norm());
        Intersection rayToAnything = intersect(rayToLight);

        // 到光源没有遮挡物，EPSILON为精度误差
        // std::cout<<temp.coords<<" "<<lightSample.coords<<" "<<(temp.coords - lightSample.coords).norm()<<std::endl;
        if (abs(rayToAnything.distance - rayToLight.t) < EPSILON)
        {
            Vector3f brdf = p.m->eval(ray.direction, rayToLight.direction, p.normal);
            float cos_theta = dotProduct(p.normal, rayToLight.direction);
            float cos_theta2 = dotProduct(-rayToLight.direction, lightSample.normal);
            float r_square = pow((lightSample.coords - p.coords).norm(), 2);
            Vector3f Ldir = lightSample.emit * brdf * cos_theta * cos_theta2 / r_square / pdf;
            // std::cout << lightSample.emit << " " << brdf << " " << cos_theta << " " << cos_theta2 << " " << r_square << " " << pdf <<" "<<Ldir<< std::endl;
            L += Ldir;
        }

        if (get_random_float() < RussianRoulette)
        {
            Ray raySampled(p.coords, p.m->sample(ray.direction, p.normal).normalized());
            Intersection inter = intersect(raySampled);
            if (inter.happened && !inter.obj->hasEmit())
            {

                Vector3f brdf = p.m->eval(ray.direction, raySampled.direction, p.normal);
                float cos_theta = dotProduct(p.normal, raySampled.direction);
                float pdf = p.m->pdf(ray.direction, raySampled.direction, p.normal);
                Vector3f shade = castRay(raySampled, depth + 1);
                Vector3f Lindir = shade * brdf * cos_theta / pdf / RussianRoulette;
                // std::cout << shade << " " << brdf << " " << cos_theta << " " << pdf << " " << Lindir << std::endl;
                L += Lindir;
            }
        }
    }
    // std::cout<<L<<std::endl;
    return L;
}