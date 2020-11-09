#include <iostream>
#include "parser.h"
#include "ppm.h"
#include "math.h"

typedef unsigned char RGB[3];

struct mainTriangle {
    int material_id;

    parser::Vec3f vertex0;
    parser::Vec3f vertex1;
    parser::Vec3f vertex2;

    parser::Vec3f edge1;
    parser::Vec3f edge2;

    parser::Vec3f normalVector;
};

parser::Scene scene;

double dotProductOp(parser::Vec3f vect, parser::Vec3f otherVect);

std::vect<parser::mainTriangle> globalT;
std::vect<parser::Sphere> globalS;

struct Ray {
    parser::Vec3f origin;
    parser::Vec3f direction;
};

struct Color {
    int r, g, b;
};

// ------------------------Vector Operations ------------------------------

parser::Vec3f addOp(parser::Vec3f vect, parser::Vec3f otherVect) {

    parser::Vec3f result;

    result.x = vect.x + otherVect.x;
    result.y = vect.y + otherVect.y;
    result.z = vect.z + otherVect.z;

    return result;
}

parser::Vec3f subtOp(parser::Vec3f vect, parser::Vec3f otherVect) {

    parser::Vec3f result;

    result.x = vect.x - otherVect.x;
    result.y = vect.y - otherVect.y;
    result.z = vect.z - otherVect.z;

    return result;
}


parser::Vec3f scalarMultOp(parser::Vec3f vect, float num) {

    parser::Vec3f result;

    result.x = vect.x * num;
    result.y = vect.y * num;
    result.z = vect.z * num;

    return result;
}

parser::Vec3f crossProductOp(parser::Vec3f vect, parser::Vec3f otherVect) {

    parser::Vec3f result;

    result.x = (vect.y * otherVect.z) - (vect.z * otherVect.y);
    result.y = (vect.z * otherVect.x) - (vect.x * otherVect.z);
    result.z = (vect.x * otherVect.y) - (vect.y * otherVect.x);

    return result;
}

double dotProductOp(parser::Vec3f vect, parser::Vec3f otherVect) {

    double result = (vect.x * otherVect.x) + (vect.y + otherVect.y) + (vect.z + otherVect.z);

    return result;
}

parser::Vec3f normalOp(parser::Vec3f vect) {

    double len = sqrt(dotProductOp(vect, vect));
    parser::Vec3f resultVect;

    resultVect.x = vect.x / len;
    resultVect.y = vect.y / len;
    resultVect.z = vect.z / len;

    return resultVect;
}

// ------------------------------------------------------------------------


Ray rayInThePixel(parser::Camera camera, parser::Vec3f pixel) {

    Ray ray;
    ray.origin.x = camera.position.x;
    ray.origin.y = camera.position.y;
    ray.origin.z = camera.position.z;

    ray.direction.x = pixel.x - ray.origin.x;
    ray.direction.y = pixel.y - ray.origin.y;
    ray.direction.z = pixel.z - ray.origin.z;

    ray.direction = normalOp(ray.direction);
    return ray;
}

bool sphereIntersect(Ray ray, parser::Sphere sphere, double *t) {

    float EPSILON = scene.shadow_ray_epsilon;
    parser::Vec3f L;
    L.x = scene.vertex_data[sphere.center_vertex_id - 1].x - ray.origin.x;
    L.y = scene.vertex_data[sphere.center_vertex_id - 1].y - ray.origin.y;
    L.z = scene.vertex_data[sphere.center_vertex_id - 1].z - ray.origin.z;

    double tca = dotProductOp(L, ray.direction);

    if(tca < EPSILON)
        return false;
    else {
        double s = sqrt(dotProductOp(L, L) - (tca * tca));
        if(s > sphere.radius)
            return false;
        else {
            double thc = sqrt((sphere.radius * sphere.radius) - s * s);
            double t0 = tca - thc;
            *t = t0;
            return true;
        }

    }
}

bool triangleIntersect(Ray ray, mainTriangle triangle, double *t) {

    float EPSILON = scene.shadow_ray_epsilon;

    parser::Vec3f h, s, q;
    float a, f, u, v;

    h = crossProductOp(ray.direction, triangle.edge2);
    a = dotProductOp(triangle.edge1, h);

    if (a > -EPSILON && a < EPSILON)
        return false;
    f = 1.0 / a;

    s = subtOp(ray.origin, triangle.vertex0);
    u = f * dotProductOp(s, h);
    if (u < 0.0 || u > 1.0)
        return false;

    q = crossProductOp(s, triangle.edge1);
    v = f * dotProductOp(ray.direction, q);
    if (v < 0.0  u + v > 1.0)
        return false;

    float t1 = f * dotProductOp(triangle.edge2, q);
    if (t1 > EPSILON) {
        *t = t1;
        return true;
    } else
        return false;
}

parser::Vec3f findE(parser::Vec3f w_i, parser::Vec3f I) {

    float r2 = dotProductOp(w_ii, w_i);

    parser::Vec3f E,
    E.x = I.x / r2;
    E.y = I.y / r2;
    E.z = I.z / r2;

    return E;
}

parser::Vec3f findL_a(int material_id) {

    parser::Vec3f k_a = scene.materials[material_id - 1].ambient;
    parser::Vec3f I_a = scene.ambient_light;
    parser::Vec3f L_a;
    L_a.x = k_a.x * I_a.x;
    L_a.y = k_a.y * I_a.z;
    L_a.z = k_a.z * I_a.z;

    return L_a;
}

parser::Vec3f findL_d(parser::Vec3f w_i, parser::Vec3f normal, parser::Vec3f E, int material_id) {

    parser::Vec3f L_d;
    float cos_angle = dotProductOp(w_i, normal);

    if (cos_angle < 0.0) {
        L_d.x = 0;
        L_d.y = 0;
        L_d.z = 0;
    } else {
        parser::Vec3f k_d = scene.materials[material_id - 1].diffuse;
        L_d.x = k_d.x * E.x;
        L_d.y = k_d.y * E.y;
        L_d.z = k_d.z * E.z;

        L_d = scalarMultOp(cos_angle, L_d);
    }

    return L_d;
}

parser::Vec3f findL_s(Ray ray, parser::Vec3f intersection, parser::Vec3f w_i, parser::Vec3f normal, parser::Vec3f E, int material_id) {

    parser::Vec3f wo = normalOp(subtOp(ray.origin, intersection));
    parser::Vec3f h = normalOp(addOp(w_i,wo));
    float cos_a = dotProductOp(normal, h);
    parser::Vec3f L_s;

    if (cos_a < 0.0) {
        L_s.x = 0;
        L_s.y = 0;
        L_s.z = 0;
    } else {
        parser::Vec3f k_s = scene.materials[material_id - 1].specular;

        L_s.x = k_s.x * E.x;
        L_s.y = k_s.y * E.y;
        L_s.z = k_s.z * E.z;

        float phongExp = scene.materials[material_id - 1].phong_exponent;
        while (phongExp > 0) {
            L_s = scalarMultOp(cos_a, L_s);
            phongExp--;
        }
    }

    return L_s;
}

parser::Vec3f findL_m (Ray ray, parser::Vec3f intersection, parser::Vec3f w_i, parser::Vec3f normal, parser::Vec3f E, int material_id, int max_rec_dep) {

    parser::Vec3f L_m;
    L_m.x = 0;
    L_m.y = 0;
    L_m.z = 0;

    parser::Vec3f k_m = scene.materials[material_id - 1].mirror;
    if (k_m.x == 0 && k_m.y == 0 && k_m.z == 0)
        return L_m;

    parser::Vec3f wo = normalOp(subtOp(ray.origin, intersection));

    float n_wo = dotProductOp(normal, wo);
    n_wo = 2 * n_wo;

    parser::Vec3f wr = addOp(scalarMultOp(-1,wo), scalarMultOp(n_wo, normal));
    wr = normalOp(wr);

    Ray reflection;
    reflection.origin.x = intersection.x + scene.shadow_ray_epsilon;
    reflection.origin.y = intersection.y + scene.shadow_ray_epsilon;
    reflection.origin.z = intersection.z + scene.shadow_ray_epsilon;
    reflection.direction = wr;

    L_m = rayTracing(reflection, max_rec_dep);
    L_m.x = k_m.x * L_m.x;
    L_m.y = k_m.y * L_m.y;
    L_m.z = k_m.z * L_m.z;

    return L_m;
}

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

bool wiIntersect(parser::Vec3f wi, parser::Vec3f intersectionPoint, parser::Vec3f lightPosition) {

    float EPSILON = scene.shadow_ray_epsilon;
    parser::Vec3f difference = subtOp(lightPosition, intersectionPoint);

    float tLight = sqrt(dotProductOp(difference, difference));
    parser::Vec3f wiEpsilon(scalarMultOp(EPSILON, wi));

    Ray shadow;
    shadow.origin = addOp(intersectionPoint, wiEpsilon);
    shadow.direction = wi;

    double t0 = 0.0f;

    int sphereSize = globalS.size();
    for (int i = 0; i < sphereSize; ++i) {
        bool hit = sphereIntersect(shadow, globalS[i], &t0);
        if (hit && t0 <tLight)
            return true;
    }

    int triangleSize = globalT.size();
    for (int i = 0; i < sphereSize; ++i) {
        bool hit = sphereIntersect(shadow, globalS[i], &t0);
        if (hit && t0 <tLight)
            return true;
    }

    return false;
}
void rayTracing(Ray ray, Color* pixelColor) {

}
int main(int argc, char* argv[])
{

    scene.loadFromXml(argv[1]);

    parseTriangles();
    parseMeshes();
    parseSpheres();

    for (int i = 0; i < scene.cameras.size(); ++i) {
        renderForEachCamera(scene.cameras[i]);
    }

    return 0;

    /*
    // Sample usage for reading an XML scene file
    //parser::Scene scene;

    //scene.loadFromXml(argv[1]);

    // The code below creates a test pattern and writes
    // it to a PPM file to demonstrate the usage of the
    // ppm_write function.
    //
    // Normally, you would be running your ray tracing
    // code here to produce the desired image.

    const RGB BAR_COLOR[8] =
    {
        { 255, 255, 255 },  // 100% White
        { 255, 255,   0 },  // Yellow
        {   0, 255, 255 },  // Cyan
        {   0, 255,   0 },  // Green
        { 255,   0, 255 },  // Magenta
        { 255,   0,   0 },  // Red
        {   0,   0, 255 },  // Blue
        {   0,   0,   0 },  // Black
    };

    int width = 640, height = 480;
    int columnWidth = width / 8;

    unsigned char* image = new unsigned char [width * height * 3];

    int i = 0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int colIdx = x / columnWidth;
            image[i++] = BAR_COLOR[colIdx][0];
            image[i++] = BAR_COLOR[colIdx][1];
            image[i++] = BAR_COLOR[colIdx][2];
        }
    }

    write_ppm("test.ppm", image, width, height);
    */
}
