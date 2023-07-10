//
//  JGL__Targettable.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/10/03.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL__Targettable_hpp
#define JGL__Targettable_hpp

#ifdef __APPLE__
#pragma GCC visibility push(default)
#endif

namespace JGL {

struct _Targettable {
	virtual int  targettableShadeState() const;
	virtual bool targetted() const { return _targetted; }
	virtual void setTarget();
	virtual void clearTarget();
protected:
	bool _targetted = false;
};

} // namespace JGL

#ifdef __APPLE__
#pragma GCC visibility pop
#endif

#endif /* JGL__Targettable_hpp */
