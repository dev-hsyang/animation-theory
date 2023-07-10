//
//  main.cpp
//  IK
//
//  Created by Hyun Joon Shin on 2021/06/09.
//

#include <iostream>
#include <JGL/JGL_Window.hpp>
#include <JGL/JGL_Aligner.hpp>
#include <JGL/JGL_Toolbar.hpp>
#include <JGL/JGL_Slider.hpp>
#include <JGL/JGL_Button.hpp>
#include "AnimView.hpp"
#include <glm/gtx/quaternion.hpp>
using namespace glm;
using namespace std;

AnimView* animView;
const float DEF_K_S = 3; // spring constant
const float K_A = 0.0; // ∞¯±‚¿˙«◊
const float K_D = 0.2; // spring damping

float Ka = 0.003;
float Ks = 2.51;
float Kd = 0.07;
const glm::vec3 g = glm::vec3(0, -980, 0);
const float EPS = 0.1;



struct Particle {
    vec3 x;
    float m;
    vec3 v;
    vec3 f;
    bool fixed = false;

    Particle(const vec3& p, float mass=0.001, const vec3& vel = vec3(0))
    : x(p), m(mass), v(vel), f(0){
    }

    void clearForce() {
        f = vec3(0);
    }
    void addForce(const vec3& force) {
        f += force;
    }
    void integrate(float dt) {
        if (fixed) v = vec3(0);
        else {
            x = x + v * dt;
            v = v + f / m * dt;
        }
    }
    void draw() const {
        drawSphere(x, 1, vec4(1, .7, 0, 1));
    }
};

struct Spring {
    Particle& a;
    Particle& b;
    float k_s;
    float r;

    Spring(Particle& one, Particle& two, float k=DEF_K_S)
        : a(one), b(two), k_s(k), r(length(a.x-b.x)){
    }

    void applyForce() {
        vec3 deltaX = a.x - b.x;
        vec3 direction = normalize(deltaX);
        vec3 deltaV = a.v - b.v;
        vec3 f = -(Ks * (length(deltaX) - r) + Kd * dot(deltaV, direction)) * (deltaX / length(deltaX));
        a.addForce(f);
        b.addForce(-f);
    }

    void draw() {
        drawCylinder(a.x, b.x, 0.4, vec4(1, .2, 0, 1));
    }


};

struct Plane {
    vec3 p;
    vec3 n;

    void draw() {
        drawQuad(p, n, vec2(1000, 1000));
    }
    bool colliding(const Particle& particle) {
        if (dot(particle.x - p, n) < EPS && dot(particle.v, n) < 0)
            return true;
        return false;
    }
    void resolveCollision(Particle& particle, float alpha = 0.6) { // π›πﬂ∞Ëºˆ alpha
        vec3 vn = dot(particle.v, n) * n;
        vec3 vt = particle.v - vn;
        particle.v = vt - alpha * vn;
    }
};

struct Sphere {
    vec3 p;
    vec3 n;

    void draw() {
        drawSphere(p, 15.f);
    }

    bool colliding(const Particle& particle) {
        if (dot(particle.x - p, n) < EPS && dot(particle.v, n) < 0)
            return false;
        return false;
    }
    void resolveCollision(Particle& particle, float alpha = 0.06) {
        vec3 vn = dot(particle.v, n) * n;
        vec3 vt = particle.v - vn;
        particle.v = vt - alpha * vn;
    }
};

vector<Plane> planes;
vector<Particle> particles;
vector<Spring> springs;
vector<Sphere> spheres;

float randf() { // 0 ~ 1 ≥≠ºˆ ª˝º∫ «‘ºˆ
    return rand() / (float)RAND_MAX;
}

void init() {
    particles.clear();
    springs.clear();
    planes.clear();
    spheres.clear();
    planes.push_back({ vec3(0), vec3(0, 1, 0) });
    spheres.push_back({ vec3(-30, 14, -5), vec3(1, 1, 1) });

    const int N = 10;
    
    for (int i = 0; i < N; i++) { // particles
        for (int j = 0; j < N; j++) {
            float rx = (randf() * 2 - 1) * 0.05;
            float ry = (randf() * 2 - 1) * 0.05;
            float rz = (randf() * 2 - 1) * 0.05;
            particles.push_back(Particle({ (j - (N - 1) / 2.f) * 5 + rx, i * 5 + 80 + ry, rz }));
        }
    }
    for (int i = 0; i < N; i++) { // horizontal springs
        for (int j = 0; j < N-1; j++) {
            springs.push_back({ particles[j + i * N], particles[j + 1 + i * N] });
        }
    }
    for (int i = 0; i < N-1; i++) { // vertical springs
        for (int j = 0; j < N; j++) {
            springs.push_back({ particles[j + i * N], particles[j + (i + 1) * N] });
        }
    }
    for (int i = 0; i < N-1; i++) { // diagonal springs
        for (int j = 0; j < N-1; j++) {
            springs.push_back({ particles[j + i * N], particles[j + 1 + (i + 1) * N] });
        }
    }
    for (int i = 0; i < N-1; i++) { // diagonal springs
        for (int j = 0; j < N-1; j++) {
            springs.push_back({ particles[j + 1 + i * N], particles[j + (i + 1) * N] });
        }
    }
    /*
    particles.push_back(Particle({ 0, 50, 0 }));
    particles.push_back(Particle({ 10, 50, 0 }));
    particles.front().fixed = true;
    springs.push_back(Spring(particles[0], particles[1]));
    */

}

float t;

void frame(float dt) {
    int step = 100;
    for (int i = 0; i < step; i++) {
        for (auto& p : particles) p.clearForce();
        for (auto& p : particles) p.addForce(-Ka * p.v);
        for (auto& p : particles) p.addForce(p.m * g);
        for (auto& s : springs) s.applyForce();
        for (auto& p : particles) {
            for (auto& plane : planes) {
                if (plane.colliding(p))
                    plane.resolveCollision(p);
            }
        }
        for (auto& p : particles) {
            for (auto& sphere : spheres) {
                if (sphere.colliding(p))
                    sphere.resolveCollision(p);
            }
        }
        for (auto& p : particles) p.integrate(dt / step);
    }
}

void render() {
    for (auto& p : particles) p.draw();
    for (auto& s : springs) s.draw();
    for (auto& s : planes) s.draw();
    for (auto& s : spheres) s.draw();
}

bool key(int k) {
    particles[0].fixed = !particles[0].fixed;
    if (k == '1')
        particles[0].fixed = false;
    
    return false;
}

bool press(const vec2& pt2, const vec3& pt3, float d) {
    return false;
}

bool drag(const vec2& pt2, const vec3& pt3, float d) {
    return false;
}

bool release(const vec2& pt2, const vec3& pt3, float d) {
    return true;
}

using namespace JGL;

int main(int argc, const char * argv[]) {
    JGL::Window* window = new JGL::Window(1100, 900, "Simulation");
    window->alignment(JGL::ALIGN_ALL);
    Aligner* a = new Aligner(0, 0, 1100, 900);
    a->type(Aligner::VERTICAL);
    a->alignment(ALIGN_ALL);
    
    /*
    Toolbar* tb = new Toolbar(0, 0, _size_toolbar_height());
    Slider<float>* s1 = new Slider<float>(0, 0, 300, _size_button_height(), "Ks");
    s1->range(0.0001, 100);
    s1->logScale(true);
    s1->autoValue(Ks);

    Slider<float>* s2 = new Slider<float>(0, 0, 300, _size_button_height(), "Kd");
    s2->range(0, 0.2);
    s2->autoValue(Kd);

    Slider<float>* s3 = new Slider<float>(0, 0, 300, _size_button_height(), "Ka");
    s3->range(0, 0.05);
    s3->autoValue(Ka);
    
    tb->end();
    */

    animView = new AnimView(0, 0, 1100, 900);
    animView->renderFunction = render;
    animView->frameFunction = frame;
    animView->initFunction = init;
    animView->pressFunc = press;
    animView->dragFunc = drag;
    animView->keyFunction = key;
    animView->releaseFunc = release;
    
    a->resizable(animView);
    
    init();
    window->show();
    JGL::_JGL::run();
    
    return 0;
}


