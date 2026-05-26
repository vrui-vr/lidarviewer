/***********************************************************************
PointShader - Class for point rendering shaders that track the current
OpenGL lighting and clipping plane state in addition to application-
defined rendering options.
Copyright (c) 2008-2026 Oliver Kreylos

This file is part of the LiDAR processing and analysis package.

The LiDAR processing and analysis package is free software; you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

The LiDAR processing and analysis package is distributed in the hope
that it will be useful, but WITHOUT ANY WARRANTY; without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the LiDAR processing and analysis package; if not, write to the
Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
02111-1307 USA
***********************************************************************/

#ifndef POINTSHADER_INCLUDED
#define POINTSHADER_INCLUDED

#include <Geometry/Plane.h>
#include <GL/gl.h>
#include <GL/GLObject.h>
#include <GL/Extensions/GLARBShaderObjects.h>

#include "Primitive.h"

class PointShader:public GLObject
	{
	/* Embedded classes: */
	public:
	enum ColorSource // Type for sources for point colors
		{
		Material, // Assigns point colors from the ambient OpenGL material property
		PointSet, // Assigns point colors from the input point set's color property
		PrimitiveDistance // Assigns point colors based on the distance from the selected primitive and a color map
		};
	
	private:
	enum DistPrimitiveType // Type for primitive distance functions
		{
		DistNone,DistPoint,DistLine,DistPlane
		};
	
	public:
	struct DataItem:public GLObject::DataItem	
		{
		friend class PointShader;
		
		/* Elements: */
		private:
		bool haveGeometryShaders; // Flag if the local OpenGL supports geometry shaders
		bool correctGamma; // Flag whether incoming point colors need to be gamma-corrected
		GLhandleARB vertexShader,fragmentShader; // Handle for the vertex and fragment shaders
		GLhandleARB geometryShader; // Handle for the optional geometry shader
		GLhandleARB programObject; // Handle for the linked program object
		bool geometryShaderAttached; // Flag whether the geometry shader is attached to the program object
		int distCenterLocation; // Location of distance calculation center point uniform variable
		int distAxisLocation; // Locations of distance calculation line end point uniform variables
		int distOffsetLocation; // Location of distance calculation offset uniform variable
		int distPlaneLocation; // Location of distance calculation plane uniform variable
		int distScaleLocation; // Location of distance calculation scale factor
		int distMapLocation; // Location of primitive distance texture map uniform variable
		int surfelScaleLocation; // Location of the surfel radii scale factor uniform variable
		GLuint distMapTexture; // Texture map ID for the distance coloring texture
		unsigned int lightStateVersion; // Version of light tracker's state reflected in the current shader program
		unsigned int clipPlaneStateVersion; // Version of clip plane tracker's state reflected in the current shader program
		unsigned int settingsVersion; // Version of other shader settings reflected in the current shader program
		
		/* Constructors and destructors: */
		DataItem(bool sCorrectGamma);
		virtual ~DataItem(void);
		
		/* Methods: */
		public:
		void setSurfelScale(GLfloat surfelScale); // Uploads the given surfel scale factor to an enabled point rendering shader
		};
	
	public:
	typedef Geometry::Plane<Primitive::Scalar,3> Plane; // Type for texture-mapping planes
	
	/* Elements: */
	private:
	ColorSource colorSource; // The requested source for point colors
	DistPrimitiveType distPrimitiveType; // Type of the currently active distance-coloring primitive
	Primitive::Point distCenter; // Center point for distance calculations
	Primitive::Vector distAxis; // Axis for line distance calculation
	Primitive::Scalar distOffset; // Offset for point or line distance calculations
	Plane distPlane; // Plane for distance calculations
	Primitive::Scalar distScale; // Scale factor for distance calculation
	ColorSource effectiveColorSource; // The effective point color source, taking into account the validity of the distance primitive
	bool useLighting; // Flag to enable point-based lighting
	bool useSurfels; // Flag whether the point renderer uses surface-aligned scaled disks to render points
	Primitive::Scalar surfelScale; // Scale factor for surfel radii
	unsigned int settingsVersion; // Version of other shader settings
	
	/* Private methods: */
	void setDistPrimitiveType(DistPrimitiveType newDistPrimitiveType); // Sets the type of distance calculation primitive
	void buildShader(GLContextData& contextData,DataItem* dataItem) const; // Rebuilds the point rendering shader based on the current states of all OpenGL light sources and clipping planes and current rendering settings
	
	/* Constructors and destructors: */
	public:
	PointShader(void); // Creates a default point rendering shader
	
	/* Methods from class GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	
	/* New methods: */
	void setColorSource(ColorSource newColorSource); // Sets the source for point colors
	void setDistancePrimitive(Primitive* newDistancePrimitive); // Sets the primitive used for subsequent point distance calculations
	void setDistanceScale(Primitive::Scalar newDistScale); // Sets the scale factor for distance-based coloring
	void setUseLighting(bool newUseLighting); // Sets the lighting flag
	void setUseSurfels(bool newUseSurfels); // Sets the surfel rendering flag
	void setSurfelScale(Primitive::Scalar newSurfelScale); // Sets the scale factor for surfel radii
	DataItem* enable(GLContextData& contextData) const; // Enables the point rendering shader in the given OpenGL context; returns a context data item to control the enabled shader
	void disable(DataItem* dataItem) const; // Disables the point rendering shader controlled through the given context data item
	};

#endif
