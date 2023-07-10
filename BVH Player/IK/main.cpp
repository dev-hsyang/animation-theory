//
//  main.cpp
//  IK
//
//  Created by Hyun Joon Shin on 2021/06/09.
//

#include <iostream>
#include <JGL/JGL_Window.hpp>
#include "AnimView.hpp"
#include <glm/gtx/quaternion.hpp>
#include <Eigen/dense>
 
glm::vec3 oldPt3;
glm::vec3 pickPt;
glm::vec3 targetPt;
float oldD;
int picked = -1;
AnimView* animView;


using namespace std;
using namespace glm;

struct Joint {
	vec3 link;
	quat q;
	int parent=-1; // joint는 parent 즉 root를 알아야함
    
    vec3 position;
    quat orientation;
    
	vec3 getPosition() const { // position : linkT * jointT * linkT * jointT ......
        return position;
	}
    
	quat getOrientation() const {
        return orientation;
	}
    
    void update(const vec3& p_parent, const quat& q_parent){
        quat l = quat(0, link); // 3vec 앞에 0 을 붙여 quat으로 , Link
        quat p = q_parent * l * inverse(q_parent); // qvq^-1
        position = p_parent + vec3(p.x, p.y, p.z); // qvq^-1 + p
        orientation = q_parent * q;
    }
};

// kinematics를 위해 articulated body 정의.
// BVH파일의 bone structure에 맞게 articulated body를 만들어야 한다.
struct Body { // Articulated body
	vector<Joint> joints;
    
	void draw() {
		for( auto& j: joints ) {
			vec3 p = j.getPosition();
			drawSphere( p, 1.0, vec4(1,0.5,0,1) ); // ( position, r, color )
			if( j.parent>=0 ) {
				vec3 pp = joints[j.parent].getPosition();
				drawCylinder(p, pp, 0.5, vec4(1,0.5,0,1) );
			}
		}
	}
    
	void emplace( const vec3& l, int p=-1 ) {
		joints.push_back( {l, quat(1,0,0,0), p} );
	}
    
    void update(){
        for(auto& j: joints){
            if(j.parent>=0){
                const Joint& p = joints[j.parent];
                j.update(p.getPosition(), p.getOrientation());
            }
            else{
                j.update(vec3(0), quat(1,0,0,0));
            }
        }
    }
    
    vector<int> ancestor(int k) const{
        vector<int> ret;
        int cur = joints[k].parent;
        
        while(cur>=0){
            ret.push_back(cur);
            cur = joints[cur].parent;
        }
        return ret;
    }
    
    void solveIK( int k, const vec3& p){
        const vector<vec3> omegas = {{1,0,0}, {0,1,0}, {0,0,1}};
        float h = 0.1;
        auto ancestors = ancestor(k);
        
        for(int iter=0; iter<10; iter++){
            Eigen::VectorXf x(3);
            vec3 dx = p - joints[k].getPosition();
            x << dx.x, dx.y, dx.z;
            
            int count = 0;
            Eigen::MatrixXf J(3, ancestors.size()*3);
            count = 0;
            
            for(auto cur : ancestors){
                vec3 pp= joints[k].getPosition() - joints[cur].getPosition();
                for(auto omega : omegas ){
                    vec3 Jcol = cross(omega, pp);
                    J.col(count++) << Jcol.x, Jcol.y, Jcol.z;
                }
            }
              
            Eigen::VectorXf dtheta = J.bdcSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(x);
            count = 0;
            for(auto cur : ancestors){
                for(auto omega: omegas){
                    joints[cur].q = exp(quat(0, dtheta[count++] * omega * 0.5f * h)) * joints[cur].q;
                }
            }
            update();
        }
    }
};

Body body;

int getPicked( const glm::vec3& pt ) {
	for( int i=0; i<body.joints.size(); i++ ) {
		if( length( body.joints[i].getPosition()-pt ) < 1.6 )
			return i;
	}
	return -1;
}

glm::vec3 getPosition( int i ) {
	return body.joints[i].getPosition();
}

void solveIK( int i, const glm::vec3& p ) {
	//body.solveIK( picked, targetPt );
}


float c = 60;


void init() {
    // walkstartA.bvh
    body.emplace(vec3(0, c, 0)); // 0 center
    body.emplace(vec3(5.07959, 0, 0), 0); // 1 lefthip
    body.emplace(vec3(0, -25.5832, 0), 1); // 2 leftknee
    body.emplace(vec3(0, -25.5523, 0), 2); // 3 leftankle
    body.emplace(vec3(0, -7.6657, 0), 3); // 4 leftheel
    body.emplace(vec3(0, -7.6657, 9.36566), 3); // 5 lefttoes
    body.emplace(vec3(0, 0, 3.83285), 5);// 6 lefttoes_end
    
    body.emplace(vec3(-5.07959, 0, 0), 0); // 7 righthip
    body.emplace(vec3(0, -25.4906, 0), 7); // 8 rightknee
    body.emplace(vec3(0, -25.7685, 0), 8); // 9 rightankle
    body.emplace(vec3(0, -7.73055, 0), 9); // 10 rightheel
    body.emplace(vec3(0, -7.73055, 9.44489), 9); // 11 righttoes
    body.emplace(vec3(0, 0, 3.86528), 11); // 12 righttoes_end
    
    body.emplace(vec3(0, 6.76249, 0), 0); // 13 Belly
    body.emplace(vec3(0, 8.41452, 0), 13); // 14 SolarPlexus
    body.emplace(vec3(0, 8.41452, 0), 14); // 15 Sternum
    body.emplace(vec3(0, 8.41452, 0), 15); // 16 Neck
    body.emplace(vec3(0, 7.07128, 0), 16); // 17 Head
    body.emplace(vec3(0, 2.12139, 0), 17); // 18 Head_end

    body.emplace(vec3(0, 8.41452, 0), 14); // 19 LeftLat
    body.emplace(vec3(0, 5.92742, 0), 19); // 20 LeftCollar
    body.emplace(vec3(7.30288, 0, 0), 20); // 21 LeftShoulder
    body.emplace(vec3(0, -18.7435, 0), 21); // 22 LeftElbow
    body.emplace(vec3(0, -14.1117, 0), 22); // 23 LeftWrist
    body.emplace(vec3(0, -4.23351, 0), 23); // 24 LeftFingers
    
    body.emplace(vec3(0, 8.41452, 0), 14); // 25 RightLat
    body.emplace(vec3(0, 5.92742, 0), 25); // 26 RightCollar
    body.emplace(vec3(-8.05941, 0, 0), 26); // 27 RightShoulder
    body.emplace(vec3(0, -19.0369, 0), 27); // 28 RightElbow
    body.emplace(vec3(0, -14.0654, 0), 28); // 29 RightWrist
    body.emplace(vec3(0, -4.21961, 0), 29); // 30 RightFingers
    
    body.update();
}

quat SLERP( const quat& x, const quat& y, float t ) {
	quat q;
    
	if( dot(x,y)< 0 )
		q = inverse(x)*-y;
	else q = inverse(x)*y;
    
	quat v = log(q);
    v.w = 0;
	return x*exp(v*t);
}

float t;
float toRadian = 3.141592/180.0;
void frame(float dt) {
    t = t + 0.01;
    if(t>2)
        t = 0;
    
    quat q = exp(quat(0, 1, 0, 0) * 0.00926369f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * 5.90644f * toRadian / 2.f) * exp(quat(0, 0, 0, 1) * -6.53245f * toRadian / 2.f) * exp(quat(0, 0, 0, 1) *  -0.319735f / 2.f) * exp(quat(0, 1, 0, 0) * -20.2656f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * 1.00927f * toRadian / 2.f);
    
    quat q2 = exp(quat(0, 0, 0, 1) * -1.6404f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * 30.0275f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * 0.820071f * toRadian / 2.f);
    quat q3 = exp(quat(0, 0, 0, 1) * 1.31771f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * 8.31828f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * 6.4178f * toRadian / 2.f);
    quat q4 = exp(quat(0, 1, 0, 0) * -20.66f * toRadian / 2.f);
    quat q5 = exp(quat(0, 1, 0, 0) * 39.3f * toRadian / 2.f);
    quat q6 = exp(quat(0, 0, 0, 1) * 0.188316f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * 31.13f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * -0.093458f * toRadian / 2.f);
    quat q7 = exp(quat(0, 0, 0, 1) * -3.99778f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * 6.57672f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * -9.11046f * toRadian / 2.f);
    quat q8 = exp(quat(0, 1, 0, 0) * -21.34f * toRadian / 2.f);
    quat q9 = exp(quat(0, 1, 0, 0) * 39.3f * toRadian / 2.f);
    quat q10 = exp(quat(0, 0, 0, 1) * -1.03893f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * 13.6083f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * 5.08306f * toRadian / 2.f);
    quat q11 = exp(quat(0, 0, 0, 1) * -2.72f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * 28.59f * toRadian / 2.f);
    quat q12 = exp(quat(0, 0, 0, 1) * 3.41521f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * -28.5199f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * 0.136573f * toRadian / 2.f);
    quat q13 = exp(quat(0, 0, 0, 1) * -16.3988f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * 25.2887f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * 1.31912e-015f * toRadian / 2.f);
    quat q14 = exp(quat(0, 0, 0, 1) * 4.67676f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * 0.499627f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * -10.0656f * toRadian / 2.f);
    quat q15 = exp(quat(0, 0, 0, 1) * 31.745f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * -3.71023f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * 10.0221f * toRadian / 2.f);
    quat q16 = exp(quat(0, 0, 0, 1) * -20.947f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * -28.0465f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * -31.7233f * toRadian / 2.f);
    quat q17 = exp(quat(0, 0, 0, 1) * -0.181993f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * -5.69f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * -0.020099f * toRadian / 2.f);
    quat q18 = exp(quat(0, 0, 0, 1) * 16.3988f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * 25.2887f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * -1.31912e-015f * toRadian / 2.f);
    quat q19 = exp(quat(0, 0, 0, 1) * 2.28488f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * 0.894973f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * 5.17683f * toRadian / 2.f);
    quat q20 = exp(quat(0, 0, 0, 1) * -30.2931f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * -2.59249f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * -7.13675f * toRadian / 2.f);
    quat q21 = exp(quat(0, 0, 0, 1) * 3.6402f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * -28.6458f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * -2.56401f * toRadian / 2.f);
    quat q22 = exp(quat(0, 0, 0, 1) * 0.150882f * toRadian / 2.f) * exp(quat(0, 1, 0, 0) * -5.04f * toRadian / 2.f) * exp(quat(0, 0, 1, 0) * 0.010039f * toRadian / 2.f);
    
    quat sq = SLERP(quat(1,0,0,0), q, t);
    quat sq2 = SLERP(quat(1,0,0,0), q2, t);
    quat sq3 = SLERP(quat(1,0,0,0), q3, t);
    quat sq4 = SLERP(quat(1,0,0,0), q4, t);
    quat sq5 = SLERP(quat(1,0,0,0), q5, t);
    quat sq6 = SLERP(quat(1,0,0,0), q6, t);
    quat sq7 = SLERP(quat(1,0,0,0), q7, t);
    quat sq8 = SLERP(quat(1,0,0,0), q8, t);
    quat sq9 = SLERP(quat(1,0,0,0), q9, t);
    quat sq10 = SLERP(quat(1,0,0,0), q10, t);
    quat sq11 = SLERP(quat(1,0,0,0), q11, t);
    quat sq12 = SLERP(quat(1,0,0,0), q12, t);
    quat sq13 = SLERP(quat(1,0,0,0), q13, t);
    quat sq14 = SLERP(quat(1,0,0,0), q14, t);
    quat sq15 = SLERP(quat(1,0,0,0), q15, t);
    quat sq16 = SLERP(quat(1,0,0,0), q16, t);
    quat sq17 = SLERP(quat(1,0,0,0), q17, t);
    quat sq18 = SLERP(quat(1,0,0,0), q18, t);
    quat sq19 = SLERP(quat(1,0,0,0), q19, t);
    quat sq20 = SLERP(quat(1,0,0,0), q20, t);
    quat sq21 = SLERP(quat(1,0,0,0), q21, t);
    quat sq22 = SLERP(quat(1,0,0,0), q22, t);
    
    body.joints[0].q = sq;
    body.joints[1].q = sq2; // left hip
    body.joints[2].q = sq3; // leftknee
    body.joints[3].q = sq4; // leftankle
    body.joints[5].q = sq5; // lefttoes
    body.joints[7].q = sq6; // righthip
    body.joints[8].q = sq7; // rightknee
    body.joints[9].q = sq8; // rightankle
    body.joints[11].q = sq9; // righttoes
    body.joints[13].q = sq10; // belly
    body.joints[16].q = sq11; // neck
    body.joints[17].q = sq12; // head
    body.joints[19].q = sq13; // leftlat
    body.joints[20].q = sq14; // leftCollar
    body.joints[21].q = sq15; // leftshoulder
    body.joints[22].q = sq16; // leftelbow
    body.joints[23].q = sq17; // leftrist
    body.joints[25].q = sq18; // rightlat
    body.joints[26].q = sq19; // rightcollar
    body.joints[27].q = sq20; // rightshoulder
    body.joints[28].q = sq21; // rightelbow
    body.joints[29].q = sq22; // rightwrist
    // Zrotation  Xrotation  Yrotation
//    0.00926369 5.90644 -6.53245 -0.319735 -20.2656 1.00927 X Y Z Z X Y CENTER
//    -1.6404 30.0275 0.820071 lefthip
//    1.31771 8.31828 6.4178 leftknee
//    0 -20.66 0 leftankle
//    0 39.3 0 lefttoes
//    0.188316 31.13 -0.093458 righthip
//    -3.99778 6.57672 -9.11046 rightknee
//    0 -21.34 0 rightankle
//    0 39.3 0 righttoes
//    -1.03893 13.6083 5.08306 belly
//    0 0 0 solarplexus
//    0 0 0 sternum
//    -2.72 28.59 0 neck
//    3.41521 -28.5199 0.136573 head
//    -16.3988 25.2887 1.31912e-015 leftlat
//    4.67676 0.499627 -10.0656 leftcollar
//    31.745 -3.71023 10.0221 leftshoulder
//    -20.947 -28.0465 -31.7233 leftelbow
//    -0.181993 -5.69 -0.020099 leftwrist
//    16.3988 25.2887 -1.31912e-015 rightlat
//    2.28488 0.894973 5.17683 rightcollar
//    -30.2931 -2.59249 -7.13675 rightshoulder
//    3.6402 -28.6458 -2.56401 rightelbow
//    0.150882 -5.04 0.010039 rightwrist
    body.update();
    // quat r = exp(quat(0,0,0,1) * 10.f/180.f*3.141592f/2.f);    z축을 중심으로, 초당 10도(degree)로 움직이는 quat
    // body.joints[1].q = r * body.joints[1].q
	/*
    t= t+0.01;
	if( t > 1 )
        t = 0;
	quat r = SLERP(quat(-1,0,0,0), quat(0.7071,0,0,0.7071), t);
	body.joints[1].q = r; // t에 따라 돌아갈 joint
    body.update();
     */
    
}

void render() {
    body.draw();
	if( picked>=0 ) {
		drawSphere( targetPt, 1.5, glm::vec4(1,1,0,.1) ); // ( p:위치, r: 반지름, color )
	}
    
	drawQuad(vec3(0,0,0), vec3(0,1,0), vec2(1000,1000)); // ( Quad의 위치, normal vec, 크기 )
}



bool press( const glm::vec2& pt2, const glm::vec3& pt3, float d ) {
	oldPt3 = pt3;
	oldD = d;
	picked = getPicked( pt3 );
	if( picked>=0 ) {
		targetPt = getPosition( picked );
		pickPt = targetPt;
		return true;
	}
	return false;
}

bool drag( const glm::vec2& pt2, const glm::vec3& pt3, float d ) {
	if( picked>=0 ) {
		targetPt = pickPt + animView->unproject( pt2, oldD ) - oldPt3;
		solveIK( picked, targetPt );
		return true;
	}
	return false;
}

bool release( const glm::vec2& pt2, const glm::vec3& pt3, float d ) {
	picked = -1;
	return true;
}

int main(int argc, const char * argv[]) {
	JGL::Window* window = new JGL::Window(1200, 800, "simulation");
	window->alignment(JGL::ALIGN_ALL);
	animView = new AnimView(0, 0, 1200, 800);
	animView->renderFunction = render;
	animView->frameFunction = frame;
	animView->initFunction = init;
	animView->pressFunc = press;
	animView->dragFunc = drag;
	animView->releaseFunc = release;

	init();
	window->show();
	JGL::_JGL::run();
	return 0;
}


