#include <aurora/Math.h>
#include <aurora/Utility.h>
#include <aurora/Vector.h>
#include <aurora/Color.h>
#include <aurora/Matrix.h>
#include <aurora/Image.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace aurora;
using namespace std;

struct Intersection {
    bool hit;
    float distance;
    size_t index;
    
    Intersection() {
        hit = false;
        distance = AURORA_INFINITY;
        size_t index = size_t(-1) ;
        
    }
    Intersection(bool hit, float distance, int index) {
        this->hit = hit;
        this->distance = distance;
        this->index = index;
    }
};

enum BSDFType {
    Light = 0,
    Diffuse = 1,
    Specular = 2,
    None = 3
};


struct BSDF {
    BSDFType type;
    Color3 color;
    
    BSDF() {}
    BSDF(BSDFType type, const Color3 & color) {
        this->type = type;
        this->color = color;
    }
};

struct Ray {
    Vector3 origin;
    Vector3 direction;
    
     Ray() {}
    Ray(const Vector3 & origin, const Vector3 & direction) {
        this->origin = origin;
        this->direction = direction;
    }
    
    Vector3 point(float distance) const {
        return origin + direction * distance;
    }
};

struct ShaderGlobals {
    Vector3 point;
    Vector3 normal;
    Vector2 uv;
    Vector3 tangentU;
    Vector3 tangentV;
    Vector3 viewDirection;
    Vector3 lightDirection;
    Vector3 lightPoint;
    Vector3 lightNormal;
    
    ShaderGlobals() {}
    ShaderGlobals(
            const Vector3 & point, const Vector3 & normal, const Vector2 & uv,
            const Vector3 & tangentU, const Vector3 & tangentV,
            const Vector3 & viewDirection, const Vector3 & lightDirection,
            const Vector3 & lightPoint, const Vector3 & lightNormal) {
        this->point = point;
        this->normal = normal;
        this->uv = uv;
        this->tangentU = tangentU;
        this->tangentV = tangentV;
        this->viewDirection = viewDirection;
        this->lightDirection = lightDirection;
        this->lightPoint = lightPoint;
        this->lightNormal = lightNormal;
    }
};

struct Shape {
    BSDF* bsdf;
    
    Shape() {}
    Shape(BSDF * bsdf) {
        this->bsdf = bsdf;
    }
    
    virtual bool intersects(const Ray & ray, Intersection & intersection) const = 0;
    virtual void calculateShaderGlobals(
        const Ray & ray, const Intersection & intersection,
        ShaderGlobals & shaderGlobals) const = 0;
    virtual float surfaceArea() const = 0;
};

struct Scene {
    std::vector<Shape*> shapes;
    
    Scene();
    Scene(const std::vector<Shape *> & shapes) {
        this->shapes = shapes;
    }
    
    bool intersects(const Ray & ray, Intersection & intersection) const {
        for (int i = 0; i < shapes.size(); i++) {
            Shape * shape = shapes[i];
            
            Intersection temp;
            shape->intersects(ray, temp);
            
            if (temp.hit && temp.distance < intersection.distance) {
                intersection.hit = temp.hit;
                intersection.distance = temp.distance;
                intersection.index = i;
            }
        }
        
        return intersection.hit;
    }
};


struct Sphere : Shape {
    Vector3 position;
    float radius;
    
    Sphere() : Shape() {}
    Sphere(const Vector3 & position, float radius, BSDF * bsdf) : Shape(bsdf) {
        this->position = position;
        this->radius = radius;
    }
    
    virtual bool intersects(const Ray & ray, Intersection & intersection) const {
        Vector3 l = position - ray.origin;
        float t = l.dot(ray.direction);
        
        if (t < 0)
            return false;
            
        float d2 = l.dot(l) - t * t;
        float r2 = radius * radius;
        
        if (d2 > r2)
            return false;
        
        float dt = std::sqrt(r2 - d2);
        
        float t0 = t - dt;
        float t1 = t + dt;
        
        if (t0 > t1)
            std::swap(t0, t1);
        
        if (t0 < 0) {
            t0 = t1;
            
            if (t0 < 0)
                return false;
        }
        
        intersection.hit = true;
        intersection.distance = t0;
        
        return true;
    }
    virtual void calculateShaderGlobals(
            const Ray & ray, const Intersection & intersection,
            ShaderGlobals & shaderGlobals) const {
        shaderGlobals.point = ray.point(intersection.distance);
        shaderGlobals.normal = (shaderGlobals.point - position).normalize();
        
        float theta = std::atan2(shaderGlobals.normal.x, shaderGlobals.normal.z);
        float phi = std::acos(shaderGlobals.normal.y);
        
        shaderGlobals.uv.x = theta * AURORA_INV_PI * 0.5;
        shaderGlobals.uv.y = phi * AURORA_INV_PI;
        
        shaderGlobals.tangentU.x = std::cos(theta);
        shaderGlobals.tangentU.y = 0;
        shaderGlobals.tangentU.z = -std::sin(theta);
        
        shaderGlobals.tangentV.x = std::sin(theta) * std::cos(phi);
        shaderGlobals.tangentV.y = -std::sin(phi);
        shaderGlobals.tangentV.z = std::cos(theta) * std::cos(phi);
        
        shaderGlobals.viewDirection = -ray.direction;
    }
    virtual float surfaceArea() const {
        return 4.0 * AURORA_PI * radius * radius;
    }
    
    virtual Vector3 lightPosition() const{
		return position;
	}
};
  
struct Vertex{
	Vector3 position;
	Vector3 normal;
	Vector2 uv;
	
	Vertex() {}
	Vertex(const Vector3 & position, const Vector3 & normal, const Vector2 & uv){
		this->position = position;
        this->normal = normal;
        this->uv = uv;
	}
	
};

struct Triangle : Shape{
	Vertex vertices[3];
		
	Triangle() {}
	Triangle(const Vertex & v0, const Vertex & v1, const Vertex & v2, BSDF * bsdf) {
	this->bsdf = bsdf;
	this->vertices[0] = vertices[0];
	this->vertices[1] = vertices[1];
	this->vertices[2] = vertices[2];
	}
	
	Intersection intersects(Ray ray)
	{
		const float EPSILON = 0.0000001;
	    Vector3 vertex0 = vertices[0].position;
	    Vector3 vertex1 = vertices[1].position;
	    Vector3 vertex2 = vertices[2].position;
	    Vector3 edge1, edge2, h, s, q;
	    
	    float a,f,u,v;
	    
	    edge1 = vertex1 - vertex0;
	    edge2 = vertex2 - vertex0;
	    h = ray.direction.cross(edge2);
	    a = edge1.dot(h);
	    
	    if (abs(a) < EPSILON)
	        return Intersection();    // This ray is parallel to this triangle.
	    f = 1.0/a;
	    s = ray.origin - vertex0;
	    u = f * s.dot(h);
	    if (u < 0.0 || u > 1.0)
	        return Intersection();
	    q = s.cross(edge1);
	    v = f * ray.direction.dot(q);
	    if (v < 0.0 || u + v > 1.0)
	        return Intersection();
	    // At this stage we can compute t to find out where the intersection point is on the line.
	    float t = f * edge2.dot(q);
	    if (t > EPSILON && t < 1/EPSILON) // ray intersection
	    {
	        //bool hit, float distance, size_t index
	        return Intersection(true,t,-1);
	    }
	    else // This means that there is a line intersection but not a ray intersection.
        return Intersection();
	}
		
	virtual bool intersects(const Ray & ray, Intersection & intersection) const {
		return false; 
    }
    virtual void calculateShaderGlobals(
        const Ray & ray, const Intersection & intersection,
        ShaderGlobals & shaderGlobals) const {}
    virtual float surfaceArea() const {
        return 0.0;
    }
};

int main(int argc, char **argv)
{
	Vertex v0;
	Vertex v1;
	Vertex v2;
	
	v0.position = Vector3(0.0, 0.0, 0.0);
	v0.normal = Vector3(0.0, 0.0, 1.0);
	v0.uv = Vector2(0.0, 0.0);
	
	v1.position = Vector3(2.0, 0.0, 0.0);
	v1.normal = Vector3(0.0, 0.0, 1.0);
	v1.uv = Vector2(1.0, 0.0);
	
	v2.position = Vector3(1.0, 2.0, 0.0);
	v2.normal = Vector3(0.0, 0.0, 1.0);
	v2.uv = Vector2(0.0, 1.0);
	
	Ray r0;
	
	r0.origin = Vector3(1.0, 1.0, 10.0);
	r0.direction = Vector3(0.0, 0.0, 10.0);
	
	Color3 red(1.0, 0.0, 0.0);
	BSDF * diffuse = new BSDF (BSDFType::Diffuse, red);
	Triangle* t = new Triangle (v0, v1, v2, diffuse);
	
	vector<Shape*> shapes = {t};
	
	Scene scene (shapes);
	
	Intersection intersection;
	
	scene.intersects(r0, intersection); 
	
	if(intersection.hit){
		cout<<"Ponto: " <<intersection.distance<<endl;
	}
	else cout<<"Sem intersecao!"<<endl;
	
	delete t;
	delete diffuse;
	
    
    return 0;
}
