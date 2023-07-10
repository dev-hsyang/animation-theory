//
//  JGL_StatusBar.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/20.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_StatusBar_hpp
#define JGL_StatusBar_hpp

#ifdef __APPLE__
#pragma GCC visibility push(default)
#endif

#include "JGL_Aligner.hpp"

namespace JGL {

struct StatusBar : public Aligner {
	StatusBar(float x,float y,float w,float h,const std::string& name="StatusBar");
	StatusBar(float x,float y,float w,const std::string& name="StatusBar");
	StatusBar(const vec2& pos,const dim2& sz,const std::string& name="StatusBar");
	StatusBar(const vec2& pos,float w,const std::string& name="StatusBar");
	
protected:
	
	virtual void drawBox(NVGcontext* vg, const glm::rect& r ) override;
	virtual void drawContents(NVGcontext* vg, const glm::rect& r, int alignment) override;
};

} // namespace JGL

#ifdef __APPLE__
#pragma GCC visibility pop
#endif

#endif /* JGL_StatusBar_hpp */
