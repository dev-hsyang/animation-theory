//
//  JGL_ImageViewer.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/18.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_ImageViewer_hpp
#define JGL_ImageViewer_hpp

#ifdef __APPLE__
#pragma GCC visibility push(default)
#endif

#include "JGL_Widget.hpp"
#include "JGL__Draw.hpp"
#include "JGL__Scrollable.hpp"
#include "JGL__Targettable.hpp"

namespace JGL {

//! \Brief The OpenGL texture image.
//! The texture Image that can be drawn by OpenGL.
struct TexImage {
	// The size and the number of channels of the current texture
	unsigned int _texWidth  = 0; //!< The width of the OpenGL allocated texture. \sa texSize()
	unsigned int _texHeight = 0; //!< The height of the OpenGL allocated texture. \sa texSize()
	unsigned int _texChannels=0; //!< The number of channnels of the OpenGL allocated texture.
	GLuint _texID = 0;           //!< The OpenGL texture ID. \sa texId()
	
	// The size of the image requested to setup
	unsigned int _imageWidth = 0; 		//!< The width of the image sent by the user. \sa imgSize()
	unsigned int _imageHeight = 0;		//!< The height of the image sent by the user. \sa imgSize()
	unsigned int _imageChannels = 0;	//!< The number of channels of the image sent by the user.
	unsigned char* _data = nullptr;		//!< The copy of the pixel data, which is GL_BYTE (unsigned). Note this will be immediately deleted when the image is sent to OpenGL.
	bool _BGR = true;					//!< The flag indicating If it is BGR image. For OpenCV compatibility. \sa isBGR()
	bool _changed = false;				//!< The flag indicating if the user-sent image is changed to the OpenGL texture image. \sa changed()
	bool _imageLocked = false;			//!< The flag indicating the user-sent image is being modified. (Required for multi-threading model)


	//! \Brief Querying the image is available.
	//! \Return true is image is set, of which size is non zero, not being updated, and not cleared.
	virtual bool imageAvailable() const { return _imageWidth>0&&_imageHeight>0&&_imageChannels>0&&!_imageLocked; }
	//! \Brief Querying the OpenGL texture ID of the image.
	//! \Return The OpenGL texture id allocated. 0 if not allcaoted.
	virtual GLuint texId() const { return _texID; }
	//! \Brief Internal function to prepare OpenGL texture for the user-sent image data. Note the data is deleted after this function.
	virtual void makeGLImage();
	//! \Brief Clearing the allocated OpenGL texture and reset all the state back to initial.
	virtual void clear();
	//! \Brief Set the user-sent image data.
	//! \Param d The pointer to the user-sent pixel data. This must be unsigned char (8bit). Note that the data will be copyed, so you must delete the original data by your self.
	//! \Param ww The width of the user-sent image.
	//! \Param hh The height of the user-sent image.
	//! \Param channels The number of channels of the user-sent image. This must be 3 or 4.
	//! \Param isBGR The flag indicating if the pixel data is in BGR form instead of RGB. The defailt value is true.
	virtual void setImage( const unsigned char* d, int ww, int hh, int channels, bool isBGR=true );
	//! \Brief Check if the user-sent data is different from the current OpenGL texture, so needs OpenGL uploading.
	//! \Return True if the user-sent image is not matching to the OpenGL texture.
	virtual bool changed() const { return _changed; }
	//! \Brief Check if the user-sent data is being updated.
	//! \Return True if the user-sent image is being updated.
	virtual bool imageLocked() const { return _imageLocked; }
	//! \Brief The current OpenGL texture size allocated.
	//! \Return The size of the current OpenGL texture allocated.
	idim2 texSize() const { return idim2(_texWidth,_texHeight); }
	//! \Brief The current user-sent image size.
	//! \Return The size of the user-sent image.
	idim2 imgSize() const { return idim2(_imageWidth,_imageHeight); }
	
	//! \Brief Utility function to generate thumbnail image from the user-sent image.
	//! Rescale the user-sent image to the desired size, which is maximum value of the resulting image in either dimension.
	//! The size of the thumbnail image in a dimension may be smaller than the desired size due to the aspect ratio.
	//! Note: This function MUST be called immediately after setImage()
	//! \Param iw The output image width.
	//! \Param ih The output image height.
	//! \Param THUMB_W The desired image width.
	//! \Param THUMB_H The desired image height.
	//! \Return The pixel data in unsigned char (8bit) RGBA form.
	unsigned char* createThumbnail( int& iw, int& ih, int THUMB_W=256, int THUMB_H=256 );
	//! \Brief Draw the image using OpenGL.
	//! Before, the opengl context must be well initialized. The blending function is not changed in this function, so you must set it properly before the call.
	//! If the OpenGL image is not allocated before calling this function (_changed==true), this function will allocate the OpenGL image by calling makeGLImage(), thus _datais also deleted.
	//! \Param viewProjection The 3D transformation due to the viewing and projection.
	//! \Param mat The 2D transformation due to model transformation.
	void draw( const glm::mat4& viewProjection, const glm::mat4& mat=glm::mat4(1) );
	//! \Brief Draw the image using OpenGL.
	//! Before, the opengl context must be well initialized. The blending function is not changed in this function, so you must set it properly before the call.
	//! If the OpenGL image is not allocated before calling this function (_changed==true), this function will allocate the OpenGL image by calling makeGLImage(), thus _datais also deleted.
	//! \Param viewProjection The 3D transformation due to the viewing and projection.
	//! \Param mat The 2D transformation due to model transformation.
	void draw( const glm::mat4& viewProjection, const glm::mat3& mat );
	//! \Brief Draw the image using OpenGL.
	//! Before, the opengl context must be well initialized. The blending function is not changed in this function, so you must set it properly before the call.
	//! If the OpenGL image is not allocated before calling this function (_changed==true), this function will allocate the OpenGL image by calling makeGLImage(), thus _datais also deleted.
	//! Essentially, this function do exactly the same as calling draw( const glm::mat4& ) with \f$ viewProjection \cdot translate(offset) \cdot scale(s)\f$.
	//! \Param viewProjection The 3D transformation due to the viewing and projection.
	//! \Param mat The 2D transformation due to model transformation.
	void draw( const glm::mat4& viewProjection, const glm::vec2& offset, float s );
	//! \Brief Draw the image using OpenGL.
	//! Before, the opengl context must be well initialized. The blending function is not changed in this function, so you must set it properly before the call.
	//! If the OpenGL image is not allocated before calling this function (_changed==true), this function will allocate the OpenGL image by calling makeGLImage(), thus _datais also deleted.
	//! \Param mat The 2D transformation to apply.
	void draw( const glm::mat3& mat = glm::mat3(1) );
	~TexImage();
	TexImage( TexImage&& a );
	TexImage();
};



struct ImageViewer: public Widget, public _Targettable {

	ImageViewer(float xx, float yy, float ww, float hh, const string& title="" );
	
	virtual void clear();
	virtual void setImage( const unsigned char* d, int ww, int hh, int channels, bool isBGR=true );

	virtual void resizableToImage( bool resizable ) { _resizableToImage = resizable; }
	virtual bool resizableToImage() const { return _resizableToImage; }
	virtual TexImage& texImage() { return _texImage; }
	virtual const TexImage& texImage() const { return _texImage; }
	virtual glm::mat4 viewProjection() const;
	void fitToScreen();
	virtual bool imageAvailable() const { return texImage().imageAvailable(); }
	template<typename T>
	glm::vec<2,T> screenToImage( const glm::vec<2,T>& pt ) const;
	template<typename T>
	glm::vec<2,T>		imageToScreen( const glm::vec<2,T>& pt ) const;
	virtual rect		getScreenRect( const glm::rect& rect ) const;
	virtual irect 		getScreenRect( const glm::irect& rect ) const;
	// Following function is only for specual purpose. Here, the scroll value is ignored and replaced with the given offset value
	template<typename T>
	glm::vec<2,T> screenToImage( const glm::vec<2,T>& pt, const glm::vec2& offset ) const;
	template<typename T>
	glm::vec<2,T>		imageToScreen( const glm::vec<2,T>& pt, const glm::vec2& offset ) const;
	virtual rect		getScreenRect( const glm::rect& rect, const glm::vec2& offset ) const;
	virtual irect 		getScreenRect( const glm::irect& rect, const glm::vec2& offset ) const;
	virtual bool 		zoomAction( const vec2& mousePt, float zoomDelta );

protected:
	
	virtual void	rearrange( NVGcontext* vg,int scaling ) override;
	virtual void	drawImage( TexImage& image, const glm::mat3& mat=glm::mat3(1) );
	virtual void	drawImage( TexImage& image, const glm::vec2& offset, float scale );

	virtual void	drawBoxOver(NVGcontext* vg,const rect&r) override;
	virtual void	drawContents(NVGcontext* vg,const rect&r,int a) override{};

	virtual void	drawGL() override;
	virtual bool	handle( int e ) override;

	virtual void	updateFitScale();

	float _minWidth = 100.f;
	float _maxWidth = 1920.f;
	float _minHeight= 100.f;
	float _maxHeight= 1300.f;

	bool _resizableToImage = false;
	
	TexImage _texImage;
	_Scrollable _scroller;
	vec2 _lastPt;
	
	float _viewScale = 1.f;	// image -> window: imageSize* scale = windowSpan
	float _desiredScale = 1.f;
	float _fitScale = 1.f;

	bool _fitted = false;
	bool _keepFitted = false;

};


template<typename T> inline glm::vec<2,T> ImageViewer::screenToImage( const glm::vec<2,T>& pt, const glm::vec2& offset ) const {
	glm::vec<2,T> pti;
	pti.x = T((pt.x+offset.x)/_viewScale);
	pti.y = T((pt.y+offset.y)/_viewScale);
	return pti;
}
template<typename T> inline glm::vec<2,T> ImageViewer::screenToImage( const glm::vec<2,T>& pt ) const {
	glm::vec<2,T> pti;
	pti.x = T((pt.x+_scroller.scrollOffset().x)/_viewScale);
	pti.y = T((pt.y+_scroller.scrollOffset().y)/_viewScale);
	return pti;
}

template<typename T> inline glm::vec<2,T> ImageViewer::imageToScreen( const glm::vec<2,T>& pt, const glm::vec2& offset ) const {
	glm::vec<2,T> pti;
	pti.x = T((pt.x)*_viewScale-offset.x);
	pti.y = T((pt.y)*_viewScale-offset.y);
	return pti;
}

template<typename T> inline glm::vec<2,T> ImageViewer::imageToScreen( const glm::vec<2,T>& pt ) const {
	glm::vec<2,T> pti;
	pti.x = T((pt.x)*_viewScale-_scroller.scrollOffset().x);
	pti.y = T((pt.y)*_viewScale-_scroller.scrollOffset().y);
	return pti;
}


} // namespace JGL

#ifdef __APPLE__
#pragma GCC visibility pop
#endif

#endif /* JGL_ImageViewer_hpp */
