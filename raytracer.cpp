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


void rayTracing(Ray ray, Color* pixelColor) {

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



parser::Vec3f mirror_sth(parser::Vec3f x /*intersection point*/, parser::Vec3f wo, parser::Vec3f n, int M_id /*material_id*/) {

    parser::Vec3f L_m;
    parser::Vec3f k_m = scene.materials[M_id].mirror;

    parser::Vec3f wr;
    wr = addOp(scalarMultOp(wo, -1), scalarMultOp(dotProductOp(wo, n), scalarMultOp(n, 2)));

    Ray mirrorRay = Ray(x + scene.shadow_ray_epsilon, wr);

    //to be continued..

}


int main(int argc, char* argv[])
{

    scene.loadFromXml(argv[1]);

    for (int cam=0; cam<scene.cameras.size(); cam++){
        ;
    }


    return 0;
}
