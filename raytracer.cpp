#include <iostream>
#include "parser.h"
#include "ppm.h"
#include "math.h"


struct Ray {
    parser::Vec3f origin;
    parser::Vec3f direction;
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


// YAZILACAK MUHTEMEL FONKILER
//
// getSpheresNormal
// ifSphereIntersect
// computeRay
// 

int main(int argc, char* argv[])
{
    
    parser::Scene scene;
    scene.loadFromXml(argv[1]);

    for (int cam=0; cam<scene.cameras.size(); cam++){
        ;
    }
    

    return 0;

    
}
