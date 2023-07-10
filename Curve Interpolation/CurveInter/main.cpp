//
//  main.cpp
//  CurveInter
//
//  Created by Hyun Joon Shin on 2021/03/24.
//  Modified by Hyun Seung Yang on 2021/4/14.

#include <iostream>
#include <JGL/JGL.hpp>
#include <JGL/JGL_Window.hpp>
#include <JGL/JGL_Widget.hpp>
#include <JGL/JGL_Options.hpp>
#include <JGL/JGL_Toolbar.hpp>
#include <JGL/JGL_Aligner.hpp>

using namespace std;
using namespace glm;


enum {
    LAGLANGIAN,
    LINEAR,
    BEZIER,
    CATMULL,
    BSPLINE,
    NATURAL,
};

enum {
    DRAW_LINES,
    DRAW_DOTS,
};

int curveType = LINEAR;
int drawType = DRAW_LINES;
bool closed = false;


std::vector<glm::vec2> srcPts;
std::vector<glm::vec2> samplePts;

glm::vec2 evaluateCurve( int curveType, const std::vector<glm::vec2>& srcPts, bool closed, size_t k, float t ) { // P 는 k 번째에서 0부터 1까지 만들어진 곡선. t 는 time
    glm::vec2 ret(0,0);
    switch( curveType ) {
        case LAGLANGIAN: {
            float T = k+t;
            std::vector<float> L(srcPts.size(),1);
            for(auto i=0; i<L.size(); i++){
                for(auto j=0; j<L.size(); j++){
                    if(i!=j){
                        L[i] *= T-j;
                        L[i] /= i-j;
                    }
                }
            }
            for(auto i=0; i<L.size(); i++){
                ret += L[i]*srcPts[i];
            }
        } break;
        case BEZIER:{
            if(k==0){
                return (1-t)*(1-t)*(1-t)*srcPts[0] + 3*t*(1-t)*(1-t)*srcPts[1] + 3*t*t*(1-t)*srcPts[2] + t*t*t*srcPts[3];
            }
            else{
                return  srcPts[3]; // srcPts의 4번째를 보낸다
            }
        }
            break;
        case CATMULL:{
            vec2 v0(0), v1(0);
            vec2 p2, p3;
            
            if(k>0){
                mat3 A(1, 0, 1, -1, 0, 1, 1, 1, 1); //t-1, 1, t+1
                {
                    vec3 b = {srcPts[k-1].x, srcPts[k].x, srcPts[k+1].x};
                    vec3 x = inverse(A) * b;
                    v0.x = x.y;
                }
                {
                    vec3 b = {srcPts[k-1].y, srcPts[k].y, srcPts[k+1].y};
                    vec3 x = inverse(A) * b;
                    v0.y = x.y;
                }
            }else{
                glm::mat3 A = {0,1,4,0,1,2,1,1,1};
                {
                    vec3 b = {srcPts[0].x, srcPts[1].x, srcPts[2].x};
                    vec3 x = inverse(A)*b;
                    v0.x = x.y;
                }
                {
                    vec3 b = {srcPts[0].y, srcPts[1].y, srcPts[2].y};
                    vec3 x = inverse(A)*b;
                    v0.y = x.y;
                }
            }
            if(k<srcPts.size()-2){
                mat3 A(0, 1, 4, 0, 1, 2, 1, 1, 1);
                {
                    vec3 b = {srcPts[k].x, srcPts[k+1].x, srcPts[k+2].x};
                    vec3 x = inverse(A) * b;
                    v1.x = 2 * x.x + x.y;
                    
                }
                {
                    vec3 b = {srcPts[k].y, srcPts[k+1].y, srcPts[k+2].y};
                    vec3 x = inverse(A) * b;
                    v1.y = 2 * x.x + x.y;
                }
            }else{
                mat3 A(4, 1, 0, -2, -1, 0, 1, 1, 1);
                {
                    vec3 b = {srcPts[k-1].x, srcPts[k].x, srcPts[k+1].x};
                    vec3 x = inverse(A) * b;
                    v1.x = x.y;
                }
                {
                    vec3 b = {srcPts[k-1].y, srcPts[k].y, srcPts[k+1].y};
                    vec3 x = inverse(A) * b;
                    v1.y = x.y;
                }
            }
            
            p2 = srcPts[k] + v0 / 3.f;
            p3 = srcPts[k+1] - v1 / 3.f;
            
            return (1-t)*(1-t)*(1-t)*srcPts[k] + 3*(1-t)*(1-t)*t*p2 + 3*(1-t)*t*t*p3 + t*t*t*srcPts[k+1];
            
            /*
            glm::vec2 v0(0), v1(0);
            if(k>0){
                v0 = (srcPts[k+1]-srcPts[k-1])/2.f;
            }else{
                glm::mat3 A = {0,1,4,0,1,2,1,1,1};
                {
                glm::vec3 b = {srcPts[0].x, srcPts[1].x, srcPts[2].x};
                glm::vec3 x = glm::inverse(A)*b;
                v0.x = x.y;
                }
                {
                glm::vec3 b = {srcPts[0].y, srcPts[1].y, srcPts[2].y};
                glm::vec3 x = glm::inverse(A)*b;
                v0.y = x.y;
                }
            }
            if(k<srcPts.size()-2)
                v1 = (srcPts[k+2]-srcPts[k])/2.f;
            
            glm::vec2 c0 = srcPts[k] + v0/3.f;
            glm::vec2 c1 = srcPts[k+1] - v1/3.f;
            return (1-t)*(1-t)*(1-t)*srcPts[k] + 3*(1-t)*(1-t)*t*c0 + 3*(1-t)*t*t*c1 + t*t*t*srcPts[k+1];
             */
        }
            break;
            
        case NATURAL: {
            size_t N = srcPts.size();
            
            // Gauss Seidel Iteration
            // Dx = b - Ux - Lx
            
            vector<vec2> D(N, vec2(0));
            
            for(int i=0; i<10; i++){
                D[0] = (3.f * (srcPts[1] - srcPts[N-1]) - D[1] - D[N-1]) / 4.f;
                D[N-1] = (3.f * (srcPts[0] - srcPts[N-2]) - D[0] - D[N-2]) / 4.f;
                for(int j=1; j<D.size()-1; j++){
                    D[j] = (3.f * (srcPts[j+1] - srcPts[j-1]) - D[j+1] - D[j-1]) / 4.f;
                }
            }
            /*
            // Natural end condition , Gauss Seidel Iteration
            vector<vec2> D(N, vec2(0));
            
            for(int i=0; i<10; i++){
                D[0] = (3.f * ( srcPts[1] - srcPts[0]) - D[1]) / 2.f;
                D[N-1] = (3.f * ( srcPts[N-1] - srcPts[N-2]) - D[N-2]) / 2.f;
                for(int j=1; j<D.size()-1; j++){
                    D[j] = (3.f * (srcPts[j+1] - srcPts[j-1]) - D[j+1] - D[j-1]) / 4.f;
                }
            }
            */
            /*
            // Jacobi Iteration
            // Dx = b - Ux - Lx
            // D[i, i] * x[i] = b[i] - x[i+1] - x[i-1] if i!=0 or i!=N-1
            // :: 4 * x[i] = b[i] - x[i+1] - x[i-1]  if i!=0 or i!=N-1
            
            vector<float> Dx, Dy;
            
            {
                vector<float> D0(N, 0); // D의 size N, x의 시작값 0
                vector<float> D1(N, 0);
            
                for(int i=0; i<10; i++){
                    vector<float>& x_n = (i%2==0)?D0:D1;
                    vector<float>& x_n1 = (i%2==0)?D1:D0;  // i 가 짝수면 D0를 x_n, D1을 x_n1로 보겠다
                    for(int j=1; j<x_n.size()-1; j++)
                        x_n1[j] = (3*(srcPts[j+1].x-srcPts[j-1].x) - x_n[j+1] - x_n[j-1])/4;
                }
                Dx = D0;
            }
            
            {
                vector<float> D0(N, 0);
                vector<float> D1(N, 0);
            
                for(int i=0; i<10; i++){
                    vector<float>& x_n = (i%2==0)?D0:D1;
                    vector<float>& x_n1 = (i%2==0)?D1:D0;  // i 가 짝수면 D0를 x_n, D1을 x_n1로 보겠다
                    for(int j=1; j<x_n.size()-1; j++)
                        x_n1[j] = (3*(srcPts[j+1].y-srcPts[j-1].y) - x_n[j+1] - x_n[j-1])/4;
                }
                Dy = D0;
            }
          
            
            vec2 a = srcPts[k];
            vec2 b = vec2(Dx[k], Dy[k]);
            vec2 c = 3.f * (srcPts[k+1] - srcPts[k]) - 2.f * vec2(Dx[k], Dy[k]) - vec2(Dx[k+1], Dy[k+1]);
            vec2 d = 2.f * (srcPts[k] - srcPts[k+1]) + vec2(Dx[k], Dy[k]) + vec2(Dx[k+1], Dy[k+1]);
             */
            
            vec2 a = srcPts[k];
            vec2 b = D[k]; //vec2(Dx[k], Dy[k]);
            vec2 c, d;
            if(k<N-1){
                c = 3.f * (srcPts[k+1] - srcPts[k]) - 2.f * D[k] - D[k+1];
                d = 2.f * (srcPts[k] - srcPts[k+1]) + D[k] + D[k+1];
            }else{
                c = 3.f * (srcPts[0] - srcPts[k]) - 2.f * D[k] - D[0];
                d = 2.f * (srcPts[k] - srcPts[0]) + D[k] + D[0];
            }
            return a + b*t + c*t*t + d*t*t*t;
        }
            break;
            
        case LINEAR :
        default: {
            //ret = (srcPts[k+1]-srcPts[k])*t + srcPts[k];
            //ret = (1-t)*srcPts[k] + t*srcPts[k+1];
            ret = glm::mix(srcPts[k], srcPts[k+1], t);
        }
    }
    return ret;
}

void updateCurve( int curveType, const std::vector<glm::vec2>& srcPts, bool closed ) {
    samplePts.clear();
    for( auto i=0; i<srcPts.size()-1; i++ ) {
        for( float t = 0; t<1; t+=0.02 ) {
            samplePts.push_back( evaluateCurve( curveType, srcPts, closed, i, t ) );
        }
    }
    if( closed ) {
        for( float t = 0; t<1; t+=0.02) {
            samplePts.push_back( evaluateCurve( curveType, srcPts, closed, srcPts.size()-1, t ) );
        }
    }
    else
        samplePts.push_back( evaluateCurve( curveType, srcPts, closed, srcPts.size()-2, 1 ) );
}


#include <JGL/nanovg/nanovg.h>


struct CurveWidget : JGL::Widget {
    CurveWidget(float x, float y, float w, float h, const std::string& title = "" )
    : JGL::Widget(x,y,w,h,title){}
    virtual void        drawBox(NVGcontext* vg, const glm::rect& r) override {
        nvgSave(vg);
        nvgBeginPath( vg );
        nvgRect( vg, r.x, r.y, r.w, r.h );
        nvgFillColor( vg, nvgRGBAf(0,0,0,1));
        nvgFill( vg );
        nvgRestore(vg);
    }
    
    virtual void drawContents(NVGcontext* vg, const glm::rect& r, int align ) override {
        nvgSave(vg);
        if( drawType == DRAW_LINES ) {
            nvgBeginPath( vg );
            nvgMoveTo( vg, samplePts[0].x, samplePts[0].y );
            for( auto i=1; i<samplePts.size(); i++ ) {
                nvgLineTo( vg, samplePts[i].x, samplePts[i].y );
            }
            nvgStrokeColor(vg, nvgRGBAf(0,.8f,1,1));
            nvgStrokeWidth(vg, 2);
            nvgStroke( vg );
        }
        else {
            nvgFillColor(vg, nvgRGBAf(0,1,.3f,1));
            nvgBeginPath( vg );
            for( auto i=0; i<samplePts.size(); i++ ) {
                nvgCircle(vg, samplePts[i].x, samplePts[i].y, 1);
            }
            nvgFill( vg );
        }

        for( auto i=0; i<srcPts.size(); i++ )
            if( i!= underPt ) {
                nvgBeginPath( vg );
                nvgCircle( vg, srcPts[i].x, srcPts[i].y, 5 );
                nvgFillColor( vg, nvgRGBAf(1,1,0,.8f));
                nvgFill( vg );
            }
        if( underPt>=0 ) {
            nvgBeginPath( vg );
            nvgCircle( vg, srcPts[underPt].x, srcPts[underPt].y, 5 );
            nvgFillColor( vg, nvgRGBAf(1,.1f,0,.8f));
            nvgFill( vg );
        }
        nvgRestore(vg);
    }
    virtual bool handle( int event ) override {
        glm::vec2 pt = JGL::_JGL::eventPt();
        switch( event ) {
            case JGL::EVENT_MOVE : {
                int oldPt = underPt;
                underPt = -1;
                for( auto i=0; i<srcPts.size(); i++ ) {
                    if( length(pt-srcPts[i])<6 )
                        underPt = i;
                }
                if( underPt!= oldPt ) {
                    redraw();
                }
            }break;
            case JGL::EVENT_PUSH : {
                if( underPt>=0 )
                    ptOffset = srcPts[underPt]-pt;
            }break;
            case JGL::EVENT_DRAG : {
                if( underPt>=0 ) {
                    srcPts[underPt] = pt+ptOffset;
                    updateCurve( curveType, srcPts, closed );
                    redraw();
                }
            }break;
        }
        return true;
    }
    int underPt = -1;
    glm::vec2 ptOffset = glm::vec2(0);
};


using namespace JGL;
CurveWidget* curveWidget;


void curveTypeCallback(Widget* w, void* ud ) {
    curveType = ((Options*)w)->value();
    updateCurve(curveType, srcPts, closed);
    
    if(curveType == NATURAL)
        closed = true;
    else
        closed = false;
     
    curveWidget->redraw();
}

void drawTypeCallback(Widget* w, void* ud ) {
    drawType = ((Options*)w)->value();
    curveWidget->redraw();
}

int main(int argc, const char * argv[]) {
    Window* window = new Window(640,480,"Curves");
    window->alignment(ALIGN_ALL);
    
    Aligner* aligner = new Aligner(0,0,window->w(), window->h());
    aligner->type(Aligner::VERTICAL);
    aligner->alignment(ALIGN_ALL);
    
    Toolbar* toolbar = new Toolbar(0,0,window->w(), _size_toolbar_height() );
    Options* curveTypes = new Options(0,0,200,_size_button_height() );
    curveTypes->add("Lagrangian");
    curveTypes->add("Linear");
    curveTypes->add("Bezier");
    curveTypes->add("Catmull");
    curveTypes->add("Bspline");
    curveTypes->add("Natural Spline");
    curveTypes->value(curveType);
    curveTypes->callback( curveTypeCallback );
    Options* drawType = new Options(0,0,200,_size_button_height() );
    drawType->add("Lines");
    drawType->add("Dots");
    drawType->value(::drawType);
    drawType->callback( drawTypeCallback );
    toolbar->end();
    
    curveWidget = new CurveWidget(0,0,window->w(), window->h()-toolbar->h());
    aligner->resizable( curveWidget );
    aligner->end();
    
    for( auto i=0; i<9; i++ ) {
        srcPts.push_back(glm::vec2((i*0.1+0.1)*curveWidget->w(),curveWidget->h()/2));
    }
    updateCurve(curveType,srcPts,closed);
    
    window->show();
    _JGL::run();
    
    return 0;
}

