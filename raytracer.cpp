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
    
    if (t_ca < 0) return false;
    
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

// mnurk - generates ray from the camera to pixel.
Ray computeRay(parser::Camera camera, parser::Vec3f s /*pixel coordinates*/) {

    Ray theRay;
    theRay.origin.x = camera.position.x;
    theRay.origin.y = camera.position.y;
    theRay.origin.z = camera.position.z;

    theRay.direction.x = s.x - camera.position.x;
    theRay.direction.y = s.y - camera.position.y;
    theRay.direction.z = s.z - camera.position.z;

    return theRay;
}

/*
parser::Vec3f computeRay(parser::Camera camera) {
    parser::Vec3f m;
    m.x = camera.position.x + camera.gaze.x * camera.near_distance;
    m.y = camera.position.y + camera.gaze.y * camera.near_distance;
    m.z = camera.position.z + camera.gaze.z * camera.near_distance;
    parser::Vec3f u = crossProductOp(camera.up, /* opposite of gaze (camera.gaze));
    parser::Vec3f q;
    q.x = m.x + ;
    q.y = camera.position.y + camera.gaze.y * camera.near_distance;
    q.z = camera.position.z + camera.gaze.z * camera.near_distance;
    //------------------
    Ray theRay;
    theRay.origin.x = camera.position.x;
    theRay.origin.y = camera.position.y;
    theRay.origin.z = camera.position.z;
    theRay.direction.x = s.x - camera.position.x;
    theRay.direction.y = s.y - camera.position.y;
    theRay.direction.z = s.z - camera.position.z;
    //--------------------
    return theRay;
}
*/


int main(int argc, char* argv[])
{
    
    scene.loadFromXml(argv[1]);

    for (int cam=0; cam<scene.cameras.size(); cam++){
        ;
    }
    

    return 0;
}
