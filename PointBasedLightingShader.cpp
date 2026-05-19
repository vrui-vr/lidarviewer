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

#include "PointBasedLightingShader.h"

#include <string>
#include <iostream>
#include <Misc/PrintInteger.h>
#include <Misc/StdError.h>
#include <GL/gl.h>
#include <GL/GLLightTracker.h>
#include <GL/GLClipPlaneTracker.h>
#include <GL/GLContext.h>
#include <GL/GLContextData.h>
#include <GL/Extensions/GLARBShaderObjects.h>
#include <GL/Extensions/GLARBVertexShader.h>
#include <GL/Extensions/GLARBGeometryShader4.h>
#include <GL/Extensions/GLARBFragmentShader.h>

#include "PlanePrimitive.h"
#include "CylinderPrimitive.h"
#include "SpherePrimitive.h"
#include "LinePrimitive.h"
#include "PointPrimitive.h"

/*****************************************
Methods of class PointBasedLightingShader:
*****************************************/

void PointBasedLightingShader::setDistPrimitiveType(PointBasedLightingShader::DistPrimitiveType newDistPrimitiveType)
	{
	/* Invalidate the shader if the primitive type changed: */
	if(distPrimitiveType!=newDistPrimitiveType)
		++settingsVersion;
	
	distPrimitiveType=newDistPrimitiveType;
	}

void PointBasedLightingShader::compileShader(void)
	{
	const GLLightTracker& lt=*(contextData.getLightTracker());
	const GLClipPlaneTracker& cpt=*(contextData.getClipPlaneTracker());
	
	std::string vertexShaderDefines;
	std::string vertexShaderFunctions;
	std::string vertexShaderMain;
	
	/* Create the main vertex shader starting boilerplate: */
	vertexShaderMain+="\
		void main()\n\
			{\n\
			/* Compute the vertex position in eye coordinates: */\n\
			vec4 vertexEc=gl_ModelViewMatrix*gl_Vertex;\n\
			\n\
			/* Compute the normal vector in eye coordinates: */\n\
			vec3 normalEc=normalize(gl_NormalMatrix*gl_Normal);\n\
			\n\
			/* Let the normal vector always point towards the eye: */\n\
			normalEc=faceforward(normalEc,normalEc,vertexEc.xyz);\n\
			\n";
	
	/* Determine the point's material properties: */
	if(distPrimitiveType!=DistNone)
		{
		switch(distPrimitiveType)
			{
			case DistPoint:
				vertexShaderDefines+="\
					uniform vec3 distCenter;\n\
					uniform float distOffset\n\
					uniform float distScale\n";
				
				vertexShaderMain+="\
					/* Calculate the distance from the primitive: */\n\
					float dist=(length(gl_Vertex.xyz-distCenter)-distOffset)*distScale;\n\
					\n";
				
				break;
			
			case DistLine:
				vertexShaderDefines+="\
					uniform vec3 distCenter;\n\
					uniform vec3 distAxis;\n\
					uniform float distOffset\n\
					uniform float distScale\n";
				
				vertexShaderMain+="\
					/* Calculate the distance from the primitive: */\n\
					float dist=(length(cross(gl_Vertex.xyz-distCenter,distAxis))-distOffset)*distScale;\n\
					\n";
				
				break;
			
			case DistPlane:
				vertexShaderDefines+="\
					uniform vec4 distPlane;\n\
					uniform float distScale\n";
				
				vertexShaderMain+="\
					/* Calculate the distance from the primitive: */\n\
					float dist=dot(gl_Vertex,distPlane)*distScale;\n\
					\n";
				
				break;
			
			default:
				;
			}
		
		/* Retrieve the point color from the distance color map: */
		vertexShaderDefines+="\
			uniform sampler1D distMap;\n";
		
		vertexShaderMain+="\
			/* Get the material properties from the primitive distance texture: */\n\
			vec4 ambient=texture1D(distMap,dist);\n\
			vec4 diffuse=ambient;\n";
		}
	else if(usePointColors)
		{
		if(correctGamma)
			{
			vertexShaderMain+="\
				/* Get the material properties from the gamma-corrected current color: */\n\
				vec4 color=vec4(pow(gl_Color.rgb,vec3(2.2)),gl_Color.a);\n\
				vec4 ambient=color;\n\
				vec4 diffuse=color;\n";
			}
		else
			{
			vertexShaderMain+="\
				/* Get the material properties from the current color: */\n\
				vec4 ambient=gl_Color;\n\
				vec4 diffuse=gl_Color;\n";
			}
		}
	else
		{
		vertexShaderMain+="\
			/* Get the material properties from the material state: */\n\
			vec4 ambient=gl_FrontMaterial.ambient;\n\
			vec4 diffuse=gl_FrontMaterial.diffuse;\n";
		}
	
	/* Assign specular material properties: */
	vertexShaderMain+="\
			vec4 specular=gl_FrontMaterial.specular;\n\
			float shininess=gl_FrontMaterial.shininess;\n\
			\n";
	
	/* Continue the main vertex shader: */
	vertexShaderMain+="\
			/* Calculate global ambient light term: */\n\
			vec4 ambientDiffuseAccum=gl_LightModel.ambient*ambient;\n\
			vec4 specularAccum=vec4(0.0,0.0,0.0,0.0);\n\
			\n\
			/* Accumulate all enabled light sources: */\n";
	
	/* Create light application functions for all enabled light sources: */
	for(int lightIndex=0;lightIndex<lt.getMaxNumLights();++lightIndex)
		if(lt.getLightState(lightIndex).isEnabled())
			{
			/* Create the light accumulation function: */
			vertexShaderFunctions+=lt.createAccumulateLightFunction(lightIndex);
			
			/* Call the light application function from the shader's main function: */
			vertexShaderMain+="\
				accumulateLight";
			char liBuffer[12];
			vertexShaderMain.append(Misc::print(lightIndex,liBuffer+11));
			vertexShaderMain+="(vertexEc,normalEc,ambient,diffuse,specular,shininess,ambientDiffuseAccum,specularAccum);\n";
			}
	
	/* Continue the main vertex shader: */
	vertexShaderMain+="\
			\n\
			/* Compute final vertex color: */\n\
			gl_FrontColor=ambientDiffuseAccum+specularAccum;\n\
			\n";
	
	/* Finish the main vertex shader: */
	if(useSplatting)
		{
		/* Create the splatting varyings: */
		vertexShaderDefines+="\
			varying vec3 normal;\n\
			varying float splatSize;\n\
			\n";
		
		vertexShaderMain+="\
				/* Pass normal vector to geometry shader: */\n\
				normal=normalEc;\n\
				splatSize=length(gl_Normal);\n\
				\n\
				/* Pass eye coordinate vertex position to geometry shader: */\n\
				gl_Position=vertexEc;\n\
				}\n";
		}
	else
		{
		/* Insert code to calculate the vertex' position relative to all user-specified clipping planes: */
		vertexShaderMain+=cpt.createCalcClipDistances("vertexEc");
		
		vertexShaderMain+="\
				/* Use standard vertex position: */\n\
				gl_Position=ftransform();\n\
				}\n";
		}
	
	/* Compile the vertex shader: */
	std::string vertexShaderSource=vertexShaderDefines+vertexShaderFunctions+vertexShaderMain;
	glCompileShaderFromString(vertexShader,vertexShaderSource.c_str());
	
	if(useSplatting)
		{
		if(!geometryShaderAttached)
			{
			/* Attach the geometry shader to the program object: */
			glAttachObjectARB(programObject,geometryShader);
			geometryShaderAttached=true;
			}
		
		/* Compile the surfel generation geometry shader: */
		std::string geometryShaderDefines="\
			#version 130\n\
			#extension GL_ARB_geometry_shader4: enable\n\
			\n\
			uniform float surfelSize;\n\
			\n\
			varying in vec3 normal[];\n\
			varying in float splatSize[];\n";
		
		std::string geometryShaderMain="\
			void main()\n\
				{\n\
				/* Calculate quad base vectors based on the eye-coordinate vertex position and normal: */\n\
				vec3 x;\n\
				if(abs(normal[0].x)<abs(normal[0].y)&&abs(normal[0].x)<abs(normal[0].z))\n\
					x=normalize(vec3(0.0,normal[0].z,-normal[0].y));\n\
				else if(abs(normal[0].y)<abs(normal[0].z))\n\
					x=normalize(vec3(normal[0].z,0.0,-normal[0].x));\n\
				else\n\
					x=normalize(vec3(normal[0].y,-normal[0].x,0.0));\n\
				x*=splatSize[0]*surfelSize*1.41421356;\n\
				vec3 y=cross(normal[0],x);\n\
				\n\
				/* Emit the quad's four vertices: */\n\
				gl_TexCoord[0].st=vec2(-1.0,-1.0);\n\
				gl_FrontColor=gl_FrontColorIn[0];\n\
				vec4 vertex0Ec=gl_PositionIn[0]+vec4(x,0.0);\n";
		geometryShaderMain+=cpt.createCalcClipDistances("vertex0Ec");
		geometryShaderMain+="\
				gl_Position=gl_ProjectionMatrix*vertex0Ec;\n\
				EmitVertex();\n\
				\n\
				gl_TexCoord[0].st=vec2(1.0,-1.0);\n\
				gl_FrontColor=gl_FrontColorIn[0];\n\
				vec4 vertex1Ec=gl_PositionIn[0]+vec4(y,0.0);\n";
		geometryShaderMain+=cpt.createCalcClipDistances("vertex1Ec");
		geometryShaderMain+="\
				gl_Position=gl_ProjectionMatrix*vertex1Ec;\n\
				EmitVertex();\n\
				\n\
				gl_TexCoord[0].st=vec2(-1.0,1.0);\n\
				gl_FrontColor=gl_FrontColorIn[0];\n\
				vec4 vertex2Ec=gl_PositionIn[0]-vec4(y,0.0);\n";
		geometryShaderMain+=cpt.createCalcClipDistances("vertex2Ec");
		geometryShaderMain+="\
				gl_Position=gl_ProjectionMatrix*vertex2Ec;\n\
				EmitVertex();\n\
				\n\
				gl_TexCoord[0].st=vec2(1.0,1.0);\n\
				gl_FrontColor=gl_FrontColorIn[0];\n\
				vec4 vertex3Ec=gl_PositionIn[0]-vec4(x,0.0);\n";
		geometryShaderMain+=cpt.createCalcClipDistances("vertex3Ec");
		geometryShaderMain+="\
				gl_Position=gl_ProjectionMatrix*vertex3Ec;\n\
				EmitVertex();\n\
				}\n";
		std::string geometryShaderSource=geometryShaderDefines+geometryShaderMain;
		glCompileShaderFromString(geometryShader,geometryShaderSource.c_str());
		
		/* Set the geometry shader's parameters: */
		glProgramParameteriARB(programObject,GL_GEOMETRY_VERTICES_OUT_ARB,4);
		glProgramParameteriARB(programObject,GL_GEOMETRY_INPUT_TYPE_ARB,GL_POINTS);
		glProgramParameteriARB(programObject,GL_GEOMETRY_OUTPUT_TYPE_ARB,GL_TRIANGLE_STRIP);
		
		/* Compile the surfel fragment shader: */
		const char* fragmentShaderSource=
			"\
			void main()\n\
				{\n\
				/* Discard fragments outside a unit-radius circle as defined by texture coordinates: */\n\
				if(dot(gl_TexCoord[0].xy,gl_TexCoord[0].xy)>1.0)\n\
					discard;\n\
				\n\
				gl_FragColor=gl_Color;\n\
				}\n";
		glCompileShaderFromString(fragmentShader,fragmentShaderSource);
		}
	else
		{
		if(geometryShaderAttached)
			{
			/* Detach the geometry shader from the program object: */
			glDetachObjectARB(programObject,geometryShader);
			geometryShaderAttached=false;
			}
		
		/* Compile the standard fragment shader: */
		const char* fragmentShaderSource=
			"\
			void main()\n\
				{\n\
				gl_FragColor=gl_Color;\n\
				}\n";
		glCompileShaderFromString(fragmentShader,fragmentShaderSource);
		}
	
	/* Link the program object: */
	glLinkProgramARB(programObject);
	
	/* Check if the program linked successfully: */
	GLint linkStatus;
	glGetObjectParameterivARB(programObject,GL_OBJECT_LINK_STATUS_ARB,&linkStatus);
	if(!linkStatus)
		{
		/* Get some more detailed information: */
		GLcharARB linkLogBuffer[2048];
		GLsizei linkLogSize;
		glGetInfoLogARB(programObject,sizeof(linkLogBuffer),&linkLogSize,linkLogBuffer);
		
		/* Signal an error: */
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Error \"%s\" while linking shader program",linkLogBuffer);
		}
	
	/* Retrieve the locations of the shader's uniform variables: */
	switch(distPrimitiveType)
		{
		case DistPoint:
			distCenterLocation=glGetUniformLocationARB(programObject,"distCenter");
			distOffsetLocation=glGetUniformLocationARB(programObject,"distOffset");
			distScaleLocation=glGetUniformLocationARB(programObject,"distScale");
			distMapLocation=glGetUniformLocationARB(programObject,"distMap");
			break;
		
		case DistLine:
			distCenterLocation=glGetUniformLocationARB(programObject,"distCenter");
			distAxisLocation=glGetUniformLocationARB(programObject,"distAxis");
			distOffsetLocation=glGetUniformLocationARB(programObject,"distOffset");
			distScaleLocation=glGetUniformLocationARB(programObject,"distScale");
			distMapLocation=glGetUniformLocationARB(programObject,"distMap");
			break;
		
		case DistPlane:
			distPlaneLocation=glGetUniformLocationARB(programObject,"distPlane");
			distScaleLocation=glGetUniformLocationARB(programObject,"distScale");
			distMapLocation=glGetUniformLocationARB(programObject,"distMap");
			break;
		
		default:
			;
		}
	
	if(useSplatting)
		{
		/* Get the locations of the uniform variables: */
		surfelSizeLocation=glGetUniformLocationARB(programObject,"surfelSize");
		}
	}

PointBasedLightingShader::PointBasedLightingShader(GLContextData& sContextData)
	:contextData(sContextData),
	 correctGamma(contextData.getContext().isNonlinear()),
	 haveGeometryShaders(false),
	 lightStateVersion(0),clipPlaneStateVersion(0),shaderSettingsVersion(0),
	 settingsVersion(1),
	 distPrimitiveType(DistNone),
	 usePointColors(false),
	 useSplatting(false),
	 vertexShader(0),fragmentShader(0),geometryShader(0),programObject(0),geometryShaderAttached(false)
	{
	/* Check for the required OpenGL extensions: */
	if(!GLARBShaderObjects::isSupported())
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"GL_ARB_shader_objects not supported");
	if(!GLARBVertexShader::isSupported())
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"GL_ARB_vertex_shader not supported");
	if(!GLARBFragmentShader::isSupported())
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"GL_ARB_fragment_shader not supported");
	
	/* Initialize the required extensions: */
	GLARBShaderObjects::initExtension();
	GLARBVertexShader::initExtension();
	GLARBFragmentShader::initExtension();
	
	/* Create the vertex and fragment shaders: */
	vertexShader=glCreateShaderObjectARB(GL_VERTEX_SHADER_ARB);
	fragmentShader=glCreateShaderObjectARB(GL_FRAGMENT_SHADER_ARB);
	
	/* Create the program object: */
	programObject=glCreateProgramObjectARB();
	glAttachObjectARB(programObject,vertexShader);
	glAttachObjectARB(programObject,fragmentShader);
	
	/* Check for the optional geometry shader extension: */
	if(GLARBGeometryShader4::isSupported())
		{
		/* Initialize the extension: */
		haveGeometryShaders=true;
		GLARBGeometryShader4::initExtension();
		
		/* Create the geometry shader: */
		geometryShader=glCreateShaderObjectARB(GL_GEOMETRY_SHADER_ARB);
		}
	}

PointBasedLightingShader::~PointBasedLightingShader(void)
	{
	glDeleteObjectARB(programObject);
	glDeleteObjectARB(vertexShader);
	if(haveGeometryShaders)
		glDeleteObjectARB(geometryShader);
	glDeleteObjectARB(fragmentShader);
	}

void PointBasedLightingShader::setDistancePrimitive(Primitive* newDistancePrimitive)
	{
	/* Determine the type of primitive and extract its distance calculation parameters: */
	PlanePrimitive* planePrim=dynamic_cast<PlanePrimitive*>(newDistancePrimitive);	
	if(planePrim!=0)
		{
		/* Extract the primitive's plane equation: */
		distPlane=planePrim->getPlane();
		distPlane.normalize();
		
		setDistPrimitiveType(DistPlane);
		return;
		}
	
	LinePrimitive* linePrim=dynamic_cast<LinePrimitive*>(newDistancePrimitive);
	if(linePrim!=0)
		{
		/* Extract the primitive's line equation: */
		distCenter=linePrim->getCenter();
		distAxis=linePrim->getAxis();
		
		/* Check if the primitive is a cylinder primitive and extract its radius if so: */
		CylinderPrimitive* cylinderPrim=dynamic_cast<CylinderPrimitive*>(newDistancePrimitive);
		distOffset=cylinderPrim!=0?cylinderPrim->getRadius():Primitive::Scalar(0);
		
		setDistPrimitiveType(DistLine);
		return;
		}
	
	PointPrimitive* pointPrim=dynamic_cast<PointPrimitive*>(newDistancePrimitive);
	if(pointPrim!=0)
		{
		/* Extract the primitive's point equation: */
		distCenter=pointPrim->getPoint();
		
		/* Check if the primitive is a sphere primitive and extract its radius if so: */
		SpherePrimitive* spherePrim=dynamic_cast<SpherePrimitive*>(newDistancePrimitive);
		distOffset=spherePrim!=0?spherePrim->getRadius():Primitive::Scalar(0);
		
		setDistPrimitiveType(DistPoint);
		return;
		}
	
	/* Disable primitive distance coloring: */
	setDistPrimitiveType(DistNone);
	}

void PointBasedLightingShader::setDistanceScale(Primitive::Scalar newDistScale)
	{
	distScale=newDistScale;
	}

void PointBasedLightingShader::setUsePointColors(bool newUsePointColors)
	{
	if(usePointColors!=newUsePointColors)
		{
		usePointColors=newUsePointColors;
		++settingsVersion;
		}
	}

void PointBasedLightingShader::setUseSplatting(bool newUseSplatting)
	{
	/* Disable splatting if geometry shaders are not supported: */
	newUseSplatting=newUseSplatting&&haveGeometryShaders;
	
	if(useSplatting!=newUseSplatting)
		{
		useSplatting=newUseSplatting;
		++settingsVersion;
		}
	}

void PointBasedLightingShader::enable(void)
	{
	try
		{
		/* Re-compile the shader if it is out of line with current state: */
		const GLLightTracker& lt=*(contextData.getLightTracker());
		const GLClipPlaneTracker& cpt=*(contextData.getClipPlaneTracker());
		
		if(lightStateVersion!=lt.getVersion()||clipPlaneStateVersion!=cpt.getVersion()||shaderSettingsVersion!=settingsVersion)
			{
			/* Rebuild the shader: */
			compileShader();
			
			/* Mark the shader as up-to-date: */
			lightStateVersion=lt.getVersion();
			clipPlaneStateVersion=cpt.getVersion();
			shaderSettingsVersion=settingsVersion;
			}
		
		/* Enable the shader: */
		glUseProgramObjectARB(programObject);
		}
	catch(const std::runtime_error& err)
		{
		std::cerr<<"Disabling lighting shader due to exception "<<err.what()<<std::endl;
		}
	}

void PointBasedLightingShader::setSurfelSize(float surfelSize)
	{
	if(useSplatting)
		{
		/* Set the surfel size uniform variable: */
		glUniformARB(surfelSizeLocation,surfelSize);
		}
	}

void PointBasedLightingShader::setDistanceMap(int textureUnit) const
	{
	/* Upload primitive distance calculation parameters: */
	switch(distPrimitiveType)
		{
		case DistPoint:
			{
			Geometry::Point<GLfloat,3> dc(distCenter);
			glUniformARB<3>(distCenterLocation,1,dc.getComponents());
			glUniformARB(distOffsetLocation,GLfloat(distOffset));
			glUniformARB(distScaleLocation,GLfloat(distScale));
			glUniformARB(distMapLocation,textureUnit);
			
			break;
			}
		
		case DistLine:
			{
			Geometry::Point<GLfloat,3> dc(distCenter);
			glUniformARB<3>(distCenterLocation,1,dc.getComponents());
			Geometry::Point<GLfloat,3> da(distAxis);
			glUniformARB<3>(distAxisLocation,1,da.getComponents());
			glUniformARB(distOffsetLocation,GLfloat(distOffset));
			glUniformARB(distScaleLocation,GLfloat(distScale));
			glUniformARB(distMapLocation,textureUnit);
			
			break;
			}
		
		case DistPlane:
			{
			GLfloat dp[4];
			for(int i=0;i<3;++i)
				dp[i]=GLfloat(distPlane.getNormal()[i]);
			dp[3]=GLfloat(-distPlane.getOffset());
			glUniformARB<4>(distPlaneLocation,1,dp);
			glUniformARB(distScaleLocation,GLfloat(distScale));
			glUniformARB(distMapLocation,textureUnit);
			
			break;
			}
		
		default:
			;
		}
	}

void PointBasedLightingShader::disable(void)
	{
	/* Disable the shader: */
	glUseProgramObjectARB(0);
	}
