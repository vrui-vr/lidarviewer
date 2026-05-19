/***********************************************************************
PointBasedLightingShader - Class to maintain a GLSL point-based lighting
shader that tracks the current OpenGL lighting state.
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

#ifndef POINTBASEDLIGHTINGSHADER_INCLUDED
#define POINTBASEDLIGHTINGSHADER_INCLUDED

#include <Geometry/Plane.h>
#include <GL/gl.h>
#include <GL/Extensions/GLARBShaderObjects.h>

#include "Primitive.h"

/* Forward declarations: */
class GLContextData;

class PointBasedLightingShader
	{
	/* Embedded classes: */
	private:
	enum DistPrimitiveType // Type for primitive distance functions
		{
		DistNone,DistPoint,DistLine,DistPlane
		};
	
	public:
	typedef Geometry::Plane<Primitive::Scalar,3> Plane; // Type for texture-mapping planes
	
	/* Elements: */
	private:
	GLContextData& contextData; // The OpenGL context with which this shader is associated
	bool correctGamma; // Flag whether incoming point colors need to be gamma-corrected
	bool haveGeometryShaders; // Flag if the local OpenGL supports geometry shaders
	unsigned int lightStateVersion; // Version of light tracker's state reflected in the current shader program
	unsigned int clipPlaneStateVersion; // Version of clip plane tracker's state reflected in the current shader program
	unsigned int shaderSettingsVersion; // Version of other shader settings reflected in the current shader program
	unsigned int settingsVersion; // Version of other shader settings
	DistPrimitiveType distPrimitiveType; // Type of the currently active distance-coloring primitive
	Primitive::Point distCenter; // Center point for distance calculations
	Primitive::Vector distAxis; // Axis for line distance calculation
	Primitive::Scalar distOffset; // Offset for point or line distance calculations
	Plane distPlane; // Plane for distance calculations
	Primitive::Scalar distScale; // Scale factor for distance calculation
	bool usePointColors; // Flag whether the point renderer uses point colors as ambient and diffuse color
	bool useSplatting; // Flag whether the point renderer uses surface-aligned point splats
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
	int surfelSizeLocation; // Location of the eye coordinate surfel radius uniform variable
	
	/* Private methods: */
	void setDistPrimitiveType(DistPrimitiveType newDistPrimitiveType); // Sets the type of distance calculation primitive
	void compileShader(void); // Recompiles the point-based lighting shader based on the current states of all OpenGL light sources and clipping planes
	
	/* Constructors and destructors: */
	public:
	PointBasedLightingShader(GLContextData& sContextData); // Creates a point-based lighting shader for the given OpenGL context
	~PointBasedLightingShader(void);
	
	/* Methods: */
	void setDistancePrimitive(Primitive* newDistancePrimitive); // Colors the point cloud by distance to the given primitive
	void setDistanceScale(Primitive::Scalar newDistScale); // Sets the scale factor for distance-based coloring
	void setUsePointColors(bool newUsePointColors); // Sets the point coloring flag
	void setUseSplatting(bool newUseSplatting); // Sets the point splatting bit
	void enable(void); // Enables point-based lighting in the current OpenGL context
	void setSurfelSize(float surfelSize); // Sets the eye-coordinate radius of surfels
	void setDistanceMap(int textureUnit) const; // Sets the primitive distance texturing texture unit index
	void disable(void); // Disables point-based lighting in the current OpenGL context
	};

#endif
