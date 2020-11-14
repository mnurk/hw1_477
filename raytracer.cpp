#include <iostream>
#include "parser.h"
#include "ppm.h"
#include "math.h"

#define EPSILON 0.0001

parser::Scene scene;

struct Ray {
    parser::Vec3f origin;
    parser::Vec3f direction;
    Ray(){
        parser::Vec3f o, d;
        o.x = o.y = o.z = d.x = d.y = d.z = 0.0;
        origin = o;
        direction = d;
    }
    Ray(parser::Vec3f o, parser::Vec3f d){origin=o; direction=d;}
};

struct Color {
    int r, g, b;
    Color(){r=g=b=0;}
    Color(int i, int j, int k){r=i; g=j; b=k;}
};


parser::Vec3f rayTracing(Ray ray, int maxRecDepth);



// add operation for two vectors
parser::Vec3f addOp(parser::Vec3f vect, parser::Vec3f otherVect) {

    parser::Vec3f result;

    result.x = vect.x + otherVect.x;
    result.y = vect.y + otherVect.y;
    result.z = vect.z + otherVect.z;

    return result;
}

// subtract operation for two vectors
parser::Vec3f subtOp(parser::Vec3f vect, parser::Vec3f otherVect) {

    parser::Vec3f result;

    result.x = vect.x - otherVect.x;
    result.y = vect.y - otherVect.y;
    result.z = vect.z - otherVect.z;

    return result;
}

// scalar multiplication operation with a vector and a constant
parser::Vec3f scalarMultOp(parser::Vec3f vect, float num) {

    parser::Vec3f result;

    result.x = vect.x * num;
    result.y = vect.y * num;
    result.z = vect.z * num;

    return result;
}

// cross product of two vectors
parser::Vec3f crossProductOp(parser::Vec3f vect, parser::Vec3f otherVect) {

    parser::Vec3f result;

    result.x = (vect.y * otherVect.z) - (vect.z * otherVect.y);
    result.y = (vect.z * otherVect.x) - (vect.x * otherVect.z);
    result.z = (vect.x * otherVect.y) - (vect.y * otherVect.x);

    return result;
}

// dot product of two vectors
double dotProductOp(parser::Vec3f vect, parser::Vec3f otherVect) {

    double result = (vect.x * otherVect.x) + (vect.y + otherVect.y) + (vect.z + otherVect.z);

    return result;
}

// func to get the normalized vector
parser::Vec3f normalOp(parser::Vec3f vect) {

    double len = sqrt(dotProductOp(vect, vect));
    parser::Vec3f resultVect;

    resultVect.x = vect.x / len;
    resultVect.y = vect.y / len;
    resultVect.z = vect.z / len;

    return resultVect;
}

// func to return the length of a 3D vector.
double lengthOfVector(parser::Vec3f vect){
    return sqrt(dotProductOp(vect, vect));
}

// func that computes irradience
parser::Vec3f findE(parser::Vec3f w_i, parser::Vec3f I) {

    float r2 = dotProductOp(w_i, w_i);

    parser::Vec3f E;
    E.x = I.x / r2;
    E.y = I.y / r2;
    E.z = I.z / r2;

    return E;
}

// func to do clamping
parser::Vec3f clamping(parser::Vec3f L) {

    if(L.x > 255)
        L.x = 255;
    if(L.x < 0)
        L.x = 0;
    if(L.y > 255)
        L.y = 255;
    if(L.y < 0)
        L.y = 0;
    if(L.z > 255)
        L.z = 255;
    if(L.z < 0)
        L.z = 0;

    return L;
}

// if ray intersects with the sphere.
// gets ray, the sphere and t(intersection point, as reference), returns bool.
bool ifSphereIntersect(Ray ray, parser::Sphere sphere, double& t){
    parser::Vec3f o = ray.origin;
    parser::Vec3f d = normalOp(ray.direction);
    parser::Vec3f center = scene.vertex_data[sphere.center_vertex_id];
    float r = sphere.radius;
    parser::Vec3f L = subtOp(center, o);
    double t_ca = dotProductOp(L, d);

    if (t_ca < EPSILON) return false;

    double d_squared = dotProductOp(L, L) - t_ca * t_ca;

    if (d_squared > r*r) return false;

    double t_hc = sqrt(r*r - d_squared);

    double t_0 = t_ca - t_hc;
    double t_1 = t_ca + t_hc;

    t = (t_0 < t_1) ? t_0 : t_1;

    return true;
}

// takes the ray, vertices of triangle and t(intersection point, as reference), returns bool.
bool ifTriangleIntersect(Ray ray, parser::Vec3f v_0, parser::Vec3f v_1, parser::Vec3f v_2, double& t){
    parser::Vec3f v0v1 = subtOp(v_1, v_0);
    parser::Vec3f v0v2 = subtOp(v_2, v_0);
    parser::Vec3f dirxside = crossProductOp(ray.direction, v0v2);
    double determinant = dotProductOp(v0v1, dirxside);

//    negative determinant => triangle is backfacing
    if (determinant < EPSILON) return false;
//    if the determinant is close to 0, the ray misses the triangle
    if (fabs(determinant) < EPSILON) return false;

    double inv_determinant = 1/determinant;

    parser::Vec3f v0_to_o = subtOp(ray.origin, v_0);
    double gamma = dotProductOp(v0_to_o, dirxside) * inv_determinant;
    if (gamma < 0) return false;
    if (gamma > 1) return false;

    parser::Vec3f xvec = crossProductOp(v0_to_o, v0v1);
    double beta = dotProductOp(ray.direction, xvec) * inv_determinant;
    if (beta < 0) return false;
    if (beta + gamma > 1) return false;

    double temp = dotProductOp(v0v2, xvec) * inv_determinant;

    if (temp < EPSILON) return false;

    t = temp;

    return true;
}

// func to generate ray from camera to any pixel given.
Ray generateRay(int i, int j, int camera_index){
    Ray theRay;
    parser::Vec3f e = scene.cameras[camera_index].position;
    parser::Vec3f v = scene.cameras[camera_index].up;
    parser::Vec3f w = scalarMultOp(scene.cameras[camera_index].gaze, -1.0);
    parser::Vec3f u = crossProductOp(w, v);
//    burada x,y,z,w'nin hangisi left, right, bottom ve top olduguna pdf'e gore karar verdim ins dogrudur.
    float l = scene.cameras[camera_index].near_plane.x, r = scene.cameras[camera_index].near_plane.y;
    float b = scene.cameras[camera_index].near_plane.z, t = scene.cameras[camera_index].near_plane.w;

    parser::Vec3f minus_w_dist = scalarMultOp(w, scene.cameras[camera_index].near_distance * -1.0);
    parser::Vec3f m = addOp(e, minus_w_dist);

    parser::Vec3f lu = scalarMultOp(u, l);
    parser::Vec3f tv = scalarMultOp(v, t);
    parser::Vec3f q = addOp(m, addOp(lu, tv));

    double s_u = (i+0.5)*((r-l)/scene.cameras[camera_index].image_width);
    double s_v = (j+0.5)*((t-b)/scene.cameras[camera_index].image_height);
    parser::Vec3f s_uxu = scalarMultOp(u, s_u);
    parser::Vec3f min_s_vxv = scalarMultOp(v, -1.0 * s_v);
    parser::Vec3f s = addOp(q, addOp(s_uxu, min_s_vxv));

    parser::Vec3f d = subtOp(s, e);
    theRay.origin = e;
    theRay.direction = normalOp(d);

    return theRay;
}


                                              //normal vector, light_id, material_id
                                    // wi is defined from the surface to the light
// hit_coord is the 3D coordinate on the surface where we operate on (x in slides). It can be computed, hopefully, using
// the main ray and t.
parser::Vec3f diffuse_shading(parser::Vec3f wi, parser::Vec3f n, int L_id, int M_id, parser::Vec3f hit_coord) {

    parser::Vec3f L_d;
    float cos_theta = dotProductOp(normalOp(wi), normalOp(n));
    if (cos_theta < 0){
        L_d.x = 0;
        L_d.y = 0;
        L_d.z = 0;
        return L_d;
    }
    
    parser::Vec3f k_d = scene.materials[M_id].diffuse;
    parser::Vec3f I = scene.point_lights[L_id].intensity;
    parser::Vec3f l_pos = scene.point_lights[L_id].position;
    parser::Vec3f r_vec = subtOp(l_pos, hit_coord);
    double r = lengthOfVector(r_vec);

    parser::Vec3f E = scalarMultOp(I, 1/(r*r));
    L_d.x = k_d.x * E.x;
    L_d.y = k_d.y * E.y;
    L_d.z = k_d.z * E.z;
    L_d = scalarMultOp(L_d, cos_theta);

    return L_d;
}

parser::Vec3f ambient_shading(int M_id) {

    parser::Vec3f L_a;
    parser::Vec3f k_a = scene.materials[M_id].ambient;
    parser::Vec3f I_a = scene.ambient_light;

    L_a.x = k_a.x * I_a.x;
    L_a.y = k_a.y * I_a.y;
    L_a.z = k_a.z * I_a.z;

    return L_a;
}

parser::Vec3f specular_shading(parser::Vec3f wi, parser::Vec3f wo, parser::Vec3f n, int L_id, int M_id, parser::Vec3f hit_coord) {

    parser::Vec3f L_s;
    parser::Vec3f half_vector = normalOp(addOp(wi, wo));
    float cos_alpha = dotProductOp(half_vector, normalOp(n));
    if (cos_alpha < 0){
        L_s.x = 0;
        L_s.y = 0;
        L_s.z = 0;
        return L_s;
    }

    parser::Vec3f k_s = scene.materials[M_id].specular;
    parser::Vec3f I = scene.point_lights[L_id].intensity;
    parser::Vec3f l_pos = scene.point_lights[L_id].position;
    parser::Vec3f r_vec = subtOp(l_pos, hit_coord);
    double r = lengthOfVector(r_vec);
    
    parser::Vec3f E = scalarMultOp(I, 1/(r*r));
    float phong_exponent = scene.materials[M_id].phong_exponent;

    cos_alpha = pow(cos_alpha, phong_exponent);

    L_s.x = k_s.x * E.x;
    L_s.y = k_s.y * E.y;
    L_s.z = k_s.z * E.z;
    L_s = scalarMultOp(L_s, cos_alpha);

    return L_s;
}


// n and wo must be unit vectors.
// wo is directed from intersection point to camera.
parser::Vec3f mirror_shading(parser::Vec3f inter_point, parser::Vec3f wo, parser::Vec3f n, int M_id, int maxRecDepth) {

    parser::Vec3f L_m;
    parser::Vec3f k_m = scene.materials[M_id].mirror;
    
    if(k_m.x == 0 && k_m.y == 0 && k_m.z == 0){
        L_m.x = 0;
        L_m.y = 0;
        L_m.z = 0;
        return L_m;
    }

    parser::Vec3f wr;
    wr = addOp(scalarMultOp(wo, -1), scalarMultOp(scalarMultOp(n, 2), dotProductOp(wo, n)));

    Ray mirrorRay;
    mirrorRay.origin.x = inter_point.x + scene.shadow_ray_epsilon;
    mirrorRay.origin.y = inter_point.y + scene.shadow_ray_epsilon;
    mirrorRay.origin.z = inter_point.z + scene.shadow_ray_epsilon;
    mirrorRay.direction = wr;

    L_m = rayTracing(mirrorRay, maxRecDepth);
    L_m.x = k_m.x * L_m.x;
    L_m.y = k_m.y * L_m.y;
    L_m.z = k_m.z * L_m.z;
    
    return L_m;
}

// generates the shadow ray from the intersection point on object to the point light given.
Ray generateShadowRay(parser::Vec3f intersect_point, parser::Vec3f light_position){
    Ray theRay;
    float eps = scene.shadow_ray_epsilon;
    parser::Vec3f wi = normalOp(subtOp(light_position, intersect_point)); /* vector from light to inter. point. */
    
    theRay.origin = addOp(intersect_point, scalarMultOp(wi, eps));
    theRay.direction = wi;
    
    return theRay;
}

bool isAreaShadow(Ray shadowRay, parser::Vec3f intersec_pt, parser::Vec3f light_pos){
    
    double t = 0;
    double dist_to_light = lengthOfVector(subtOp(light_pos, intersec_pt));
    
    for (int i=0; i<scene.spheres.size(); i++){
        bool intrsct = ifSphereIntersect(shadowRay, scene.spheres[i], t);
        
        if (intrsct && t < dist_to_light)
            return true;
        
    }
    
    for (int i=0; i<scene.triangles.size(); i++){
        parser::Vec3f v0 = scene.vertex_data[scene.triangles[i].indices.v0_id];
        parser::Vec3f v1 = scene.vertex_data[scene.triangles[i].indices.v1_id];
        parser::Vec3f v2 = scene.vertex_data[scene.triangles[i].indices.v2_id];
        
        bool intrsct = ifTriangleIntersect(shadowRay, v0, v1, v2, t);
        
        if (intrsct && t < dist_to_light)
            return  true;
        
    }
    
    
    for (int i=0; scene.meshes.size(); i++)
    for (int j=0; scene.meshes[i].faces.size(); j++){
        parser::Vec3f v0 = scene.vertex_data[scene.meshes[i].faces[j].v0_id];
        parser::Vec3f v1 = scene.vertex_data[scene.meshes[i].faces[j].v1_id];
        parser::Vec3f v2 = scene.vertex_data[scene.meshes[i].faces[j].v2_id];
        
        bool intrsct = ifTriangleIntersect(shadowRay, v0, v1, v2, t);
        
        if (intrsct && t < dist_to_light)
            return true;
        
    }
    
    return false;
}

parser::Vec3f rayTracing(Ray ray, int maxRecDepth) {
    parser::Vec3f L;
    
    if (maxRecDepth == 0){
        L.x = L.y = L.z = 0;
        return L;
    }
    maxRecDepth--;
    
    double t_min = std::numeric_limits<double>::max();
    double t0;
    parser::Face* tri_intersec = NULL;
    parser::Sphere* sph_intersec = NULL;
    parser::Vec3f intersec_point;
    parser::Vec3f normal_vec;
    int mat_id = 0;
    
//    check intersection with spere.
    for (int i=0; i<scene.spheres.size(); i++){
        bool intrsct = ifSphereIntersect(ray, scene.spheres[i], t0);
        
        if (intrsct && t0 < t_min){
            t_min = t0;
            sph_intersec = &(scene.spheres[i]);
            mat_id = scene.spheres[i].material_id;
        }
    }
    
//    check intersection with triangles.
    for (int i=0; i<scene.triangles.size(); i++){
        parser::Vec3f v0 = scene.vertex_data[scene.triangles[i].indices.v0_id];
        parser::Vec3f v1 = scene.vertex_data[scene.triangles[i].indices.v1_id];
        parser::Vec3f v2 = scene.vertex_data[scene.triangles[i].indices.v2_id];
        
        bool intrsct = ifTriangleIntersect(ray, v0, v1, v2, t0);
        
        if (intrsct && t0 < t_min){
            t_min = t0;
            tri_intersec = &(scene.triangles[i].indices);
            mat_id = scene.triangles[i].material_id;
            sph_intersec = NULL;
        }
        
    }
    
//    check intersection with meshes.
//    std::cout << scene.meshes.size() << std::endl;
    for (int i=0; i < scene.meshes.size(); i++){
        for (int j=0; j < scene.meshes[i].faces.size(); j++){
            
            parser::Vec3f v0 = scene.vertex_data[scene.meshes[i].faces[j].v0_id];
            parser::Vec3f v1 = scene.vertex_data[scene.meshes[i].faces[j].v1_id];
            parser::Vec3f v2 = scene.vertex_data[scene.meshes[i].faces[j].v2_id];
            
            bool intrsct = ifTriangleIntersect(ray, v0, v1, v2, t0);
            
            if (intrsct && t0 < t_min){
                t_min = t0;
                tri_intersec = &(scene.meshes[i].faces[j]);
                mat_id = scene.meshes[i].material_id;
                sph_intersec = NULL;
            }
            
        }
    }
    
    if (sph_intersec != NULL && tri_intersec == NULL){
        L = ambient_shading(mat_id);
        normal_vec = normalOp(subtOp(intersec_point, scene.vertex_data[sph_intersec->center_vertex_id]));
        parser::Vec3f wo = normalOp(subtOp(ray.origin, intersec_point));
        
        for (int i=0; i<scene.point_lights.size(); i++){
            intersec_point = addOp(ray.origin, scalarMultOp(ray.direction, t_min));
            Ray shadowRay = generateShadowRay(intersec_point, scene.point_lights[i].position);

            if (isAreaShadow(shadowRay, intersec_point, scene.point_lights[i].position)){
                continue;
            }else{
                parser::Vec3f L_s, L_d;
                parser::Vec3f wi = normalOp(subtOp(scene.point_lights[i].position, intersec_point));
                
                L_d = diffuse_shading(wi, normal_vec, i, mat_id, intersec_point);
                L_s = specular_shading(wi, wo, normal_vec, i, mat_id, intersec_point);
                
                L = addOp(L, addOp(L_s, L_d));
            }
            
        }
        
        L = addOp(L, mirror_shading(intersec_point, wo, normal_vec, mat_id, maxRecDepth));
        
        
    }else if (sph_intersec == NULL && tri_intersec != NULL){
        L = ambient_shading(mat_id);
        parser::Vec3f wo = normalOp(subtOp(ray.origin, intersec_point));
        parser::Vec3f v0 = scene.vertex_data[tri_intersec->v0_id];
        parser::Vec3f v1 = scene.vertex_data[tri_intersec->v1_id];
        parser::Vec3f v2 = scene.vertex_data[tri_intersec->v2_id];
        parser::Vec3f v0v1 = subtOp(v1, v0);
        parser::Vec3f v0v2 = subtOp(v2, v0);
        normal_vec = crossProductOp(v0v1, v0v2);
        
        for (int i=0; i<scene.point_lights.size(); i++){
            intersec_point = addOp(ray.origin, scalarMultOp(ray.direction, t_min));
            Ray shadowRay = generateShadowRay(intersec_point, scene.point_lights[i].position);
            
            if (isAreaShadow(shadowRay, intersec_point, scene.point_lights[i].position)){
                continue;
            }else{
                parser::Vec3f L_s, L_d;
                parser::Vec3f wi = normalOp(subtOp(scene.point_lights[i].position, intersec_point));
                
                L_d = diffuse_shading(wi, normal_vec, i, mat_id, intersec_point);
                L_s = specular_shading(wi, wo, normal_vec, i, mat_id, intersec_point);
                
                L = addOp(L, addOp(L_s, L_d));
            }
            
        }
        
        L = addOp(L, mirror_shading(intersec_point, wo, normal_vec, mat_id, maxRecDepth));
        
        
    }else{
        L.x = scene.background_color.x;
        L.y = scene.background_color.y;
        L.z = scene.background_color.z;
    }
    
    return L;
    
}



int main(int argc, char* argv[])
{

    scene.loadFromXml(argv[1]);

    for (int cam=0; cam<scene.cameras.size(); cam++){
        
        int k=0;
        unsigned char* image = new unsigned char [scene.cameras[cam].image_width * scene.cameras[cam].image_height * 3];
        
        for (int i=0; i<scene.cameras[cam].image_height; i++){
            for (int j=0; j<scene.cameras[cam].image_width; j++){
                Ray theRay = generateRay(i, j, cam);
                parser::Vec3f pix_color = rayTracing(theRay, scene.max_recursion_depth+1);
                pix_color = clamping(pix_color);
                
                image[k++] = pix_color.x;
                image[k++] = pix_color.y;
                image[k++] = pix_color.z;
                
            }
        }
        
        write_ppm(scene.cameras[cam].image_name.c_str(), image, scene.cameras[cam].image_width, scene.cameras[cam].image_height);
        
    }


    return 0;
}
