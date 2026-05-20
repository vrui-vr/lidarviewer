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

#include "PointShader.h"

#include <string>
#include <iostream>
#include <Misc/PrintInteger.h>
#include <Misc/StdError.h>
#include <GL/gl.h>
#include <GL/GLColor.h>
#include <GL/GLLightTracker.h>
#include <GL/GLClipPlaneTracker.h>
#include <GL/GLContext.h>
#include <GL/GLContextData.h>
#include <GL/Extensions/GLARBShaderObjects.h>
#include <GL/Extensions/GLARBVertexShader.h>
#include <GL/Extensions/GLARBGeometryShader4.h>
#include <GL/Extensions/GLARBFragmentShader.h>
#include <GL/Extensions/GLARBMultitexture.h>

#include "PlanePrimitive.h"
#include "CylinderPrimitive.h"
#include "SpherePrimitive.h"
#include "LinePrimitive.h"
#include "PointPrimitive.h"

/**************************************
Methods of class PointShader::DataItem:
**************************************/

PointShader::DataItem::DataItem(bool sCorrectGamma)
	:haveGeometryShaders(GLARBGeometryShader4::isSupported()),
	 correctGamma(sCorrectGamma),
	 vertexShader(0),fragmentShader(0),geometryShader(0),programObject(0),geometryShaderAttached(false),
	 distMapTexture(0),
	 lightStateVersion(0),clipPlaneStateVersion(0),settingsVersion(0)
	{
	/* Initialize the required OpenGL extensions: */
	GLARBShaderObjects::initExtension();
	GLARBVertexShader::initExtension();
	GLARBFragmentShader::initExtension();
	GLARBMultitexture::initExtension();
	
	/* Create the vertex and fragment shaders: */
	vertexShader=glCreateShaderObjectARB(GL_VERTEX_SHADER_ARB);
	fragmentShader=glCreateShaderObjectARB(GL_FRAGMENT_SHADER_ARB);
	
	/* Create the program object: */
	programObject=glCreateProgramObjectARB();
	glAttachObjectARB(programObject,vertexShader);
	glAttachObjectARB(programObject,fragmentShader);
	
	/* Check for the optional geometry shader extension: */
	if(haveGeometryShaders)
		{
		/* Initialize the geometry shader extension: */
		GLARBGeometryShader4::initExtension();
		
		/* Create the geometry shader: */
		geometryShader=glCreateShaderObjectARB(GL_GEOMETRY_SHADER_ARB);
		}
	
	/* Create the distance map texture: */
	glGenTextures(1,&distMapTexture);
	}

PointShader::DataItem::~DataItem(void)
	{
	/* Destroy all shaders and program objects: */
	glDeleteObjectARB(programObject);
	glDeleteObjectARB(vertexShader);
	if(haveGeometryShaders)
		glDeleteObjectARB(geometryShader);
	glDeleteObjectARB(fragmentShader);
	
	/* Create the distance map texture: */
	glDeleteTextures(1,&distMapTexture);
	}

void PointShader::DataItem::setSurfelScale(GLfloat surfelScale)
	{
	if(surfelScaleLocation>=0)
		glUniformARB(surfelScaleLocation,surfelScale);
	}

/****************************
Methods of class PointShader:
****************************/

void PointShader::setDistPrimitiveType(PointShader::DistPrimitiveType newDistPrimitiveType)
	{
	/* Invalidate the shader if the primitive type changed: */
	if(distPrimitiveType!=newDistPrimitiveType)
		++settingsVersion;
	
	distPrimitiveType=newDistPrimitiveType;
	}

void PointShader::buildShader(GLContextData& contextData,PointShader::DataItem* dataItem) const
	{
	const GLLightTracker& lt=*(contextData.getLightTracker());
	const GLClipPlaneTracker& cpt=*(contextData.getClipPlaneTracker());
	
	std::string vertexShaderDefines;
	std::string vertexShaderFunctions;
	std::string vertexShaderMain;
	
	/* Create the main vertex shader starting boilerplate: */
	vertexShaderMain+="\
		void main()\n\
			{\n";
	
	/* Check if the shader requires the vertex position in eye coordinates: */
	bool haveClipPlanes=cpt.getNumEnabledClipPlanes()>0;
	bool useSurfelsHere=useSurfels&&dataItem->haveGeometryShaders;
	if(useLighting||useSurfelsHere||haveClipPlanes)
		vertexShaderMain+="\
			/* Compute the vertex position in eye coordinates: */\n\
			vec4 vertexEc=gl_ModelViewMatrix*gl_Vertex;\n\
			\n";
	
	/* Check if the shader requires the vertex normal in eye coordinates: */
	if(useLighting||useSurfelsHere)
		vertexShaderMain+="\
			/* Compute the normal vector in eye coordinates: */\n\
			vec3 normalEc=normalize(gl_NormalMatrix*gl_Normal);\n\
			\n\
			/* Let the normal vector always point towards the eye: */\n\
			normalEc=faceforward(normalEc,normalEc,vertexEc.xyz);\n\
			\n";
	
	/* Insert code to calculate the vertex' position relative to all user-specified clipping planes if needed: */
	if(haveClipPlanes&&!useSurfelsHere)
		{
		vertexShaderMain+=cpt.createCalcClipDistances("vertexEc");
		vertexShaderMain+="\n";
		}
	
	/* Determine the point's color: */
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
					float dist=(length(gl_Vertex.xyz-distCenter)-distOffset)*distScale;\n";
				
				break;
			
			case DistLine:
				vertexShaderDefines+="\
					uniform vec3 distCenter;\n\
					uniform vec3 distAxis;\n\
					uniform float distOffset\n\
					uniform float distScale\n";
				
				vertexShaderMain+="\
					/* Calculate the distance from the primitive: */\n\
					float dist=(length(cross(gl_Vertex.xyz-distCenter,distAxis))-distOffset)*distScale;\n";
				
				break;
			
			case DistPlane:
				vertexShaderDefines+="\
					uniform vec4 distPlane;\n\
					uniform float distScale\n";
				
				vertexShaderMain+="\
					/* Calculate the distance from the primitive: */\n\
					float dist=dot(gl_Vertex,distPlane)*distScale;\n";
				
				break;
			
			default:
				;
			}
		
		/* Retrieve the point color from the distance color map: */
		vertexShaderDefines+="\
			uniform sampler1D distMap;\n";
		
		vertexShaderMain+="\
			\n\
			/* Get the material properties from the primitive distance texture: */\n\
			vec4 ambient=texture1D(distMap,dist+0.5);\n\
			vec4 diffuse=ambient;\n";
		}
	else if(usePointColors)
		{
		if(dataItem->correctGamma)
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
	
	if(useLighting)
		{
		/* Assign specular material properties: */
		vertexShaderMain+="\
			vec4 specular=gl_FrontMaterial.specular;\n\
			float shininess=gl_FrontMaterial.shininess;\n\
			\n";
		
		/* Start the lighting calculation: */
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
		
		/* Finish the lighting calculation: */
		vertexShaderMain+="\
			\n\
			/* Compute final vertex color: */\n\
			gl_FrontColor=ambientDiffuseAccum+specularAccum;\n\
			\n";
		}
	else
		{
		vertexShaderMain+="\
			\n\
			/* Assign the ambient color: */\n\
			gl_FrontColor=ambient;\n\
			\n";
		}
	
	if(useSurfelsHere)
		{
		/* Create the surfel rendering varyings: */
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
		vertexShaderMain+="\
			/* Use standard vertex position: */\n\
			gl_Position=ftransform();\n\
			}\n";
		}
	
	/* Compile the vertex shader: */
	std::string vertexShaderSource=vertexShaderDefines+vertexShaderFunctions+vertexShaderMain;
	glCompileShaderFromString(dataItem->vertexShader,vertexShaderSource.c_str());
	
	if(useSurfelsHere)
		{
		if(!dataItem->geometryShaderAttached)
			{
			/* Attach the geometry shader to the program object: */
			glAttachObjectARB(dataItem->programObject,dataItem->geometryShader);
			dataItem->geometryShaderAttached=true;
			}
		
		/* Compile the surfel generation geometry shader: */
		std::string geometryShaderDefines="\
			#version 130\n\
			#extension GL_ARB_geometry_shader4: enable\n\
			\n\
			uniform float surfelScale;\n\
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
				x*=splatSize[0]*surfelScale*1.41421356;\n\
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
		glCompileShaderFromString(dataItem->geometryShader,geometryShaderSource.c_str());
		
		/* Set the geometry shader's parameters: */
		glProgramParameteriARB(dataItem->programObject,GL_GEOMETRY_VERTICES_OUT_ARB,4);
		glProgramParameteriARB(dataItem->programObject,GL_GEOMETRY_INPUT_TYPE_ARB,GL_POINTS);
		glProgramParameteriARB(dataItem->programObject,GL_GEOMETRY_OUTPUT_TYPE_ARB,GL_TRIANGLE_STRIP);
		
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
		glCompileShaderFromString(dataItem->fragmentShader,fragmentShaderSource);
		}
	else
		{
		if(dataItem->geometryShaderAttached)
			{
			/* Detach the geometry shader from the program object: */
			glDetachObjectARB(dataItem->programObject,dataItem->geometryShader);
			dataItem->geometryShaderAttached=false;
			}
		
		/* Compile the standard fragment shader: */
		const char* fragmentShaderSource=
			"\
			void main()\n\
				{\n\
				gl_FragColor=gl_Color;\n\
				}\n";
		glCompileShaderFromString(dataItem->fragmentShader,fragmentShaderSource);
		}
	
	/* Link the program object: */
	glLinkProgramARB(dataItem->programObject);
	
	/* Check if the program linked successfully: */
	GLint linkStatus;
	glGetObjectParameterivARB(dataItem->programObject,GL_OBJECT_LINK_STATUS_ARB,&linkStatus);
	if(!linkStatus)
		{
		/* Get some more detailed information: */
		GLcharARB linkLogBuffer[2048];
		GLsizei linkLogSize;
		glGetInfoLogARB(dataItem->programObject,sizeof(linkLogBuffer),&linkLogSize,linkLogBuffer);
		
		/* Signal an error: */
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Error \"%s\" while linking shader program",linkLogBuffer);
		}
	
	/* Retrieve the locations of the shader's uniform variables: */
	switch(distPrimitiveType)
		{
		case DistPoint:
			dataItem->distCenterLocation=glGetUniformLocationARB(dataItem->programObject,"distCenter");
			dataItem->distOffsetLocation=glGetUniformLocationARB(dataItem->programObject,"distOffset");
			dataItem->distScaleLocation=glGetUniformLocationARB(dataItem->programObject,"distScale");
			dataItem->distMapLocation=glGetUniformLocationARB(dataItem->programObject,"distMap");
			break;
		
		case DistLine:
			dataItem->distCenterLocation=glGetUniformLocationARB(dataItem->programObject,"distCenter");
			dataItem->distAxisLocation=glGetUniformLocationARB(dataItem->programObject,"distAxis");
			dataItem->distOffsetLocation=glGetUniformLocationARB(dataItem->programObject,"distOffset");
			dataItem->distScaleLocation=glGetUniformLocationARB(dataItem->programObject,"distScale");
			dataItem->distMapLocation=glGetUniformLocationARB(dataItem->programObject,"distMap");
			break;
		
		case DistPlane:
			dataItem->distPlaneLocation=glGetUniformLocationARB(dataItem->programObject,"distPlane");
			dataItem->distScaleLocation=glGetUniformLocationARB(dataItem->programObject,"distScale");
			dataItem->distMapLocation=glGetUniformLocationARB(dataItem->programObject,"distMap");
			break;
		
		default:
			;
		}
	
	if(useSurfelsHere)
		{
		/* Get the locations of the uniform variables: */
		dataItem->surfelScaleLocation=glGetUniformLocationARB(dataItem->programObject,"surfelScale");
		}
	else
		{
		/* Disable surfels: */
		dataItem->surfelScaleLocation=-1;
		}
	
	/* Mark the shader as up-to-date: */
	dataItem->lightStateVersion=lt.getVersion();
	dataItem->clipPlaneStateVersion=cpt.getVersion();
	dataItem->settingsVersion=settingsVersion;
	}

PointShader::PointShader(void)
	:distPrimitiveType(DistNone),
	 useLighting(false),usePointColors(true),useSurfels(false),surfelScale(1),
	 settingsVersion(1)
	{
	}

void PointShader::initContext(GLContextData& contextData) const
	{
	/* Create a data item and store it in the given OpenGL context: */
	DataItem* dataItem=new DataItem(contextData.getContext().isNonlinear());
	contextData.addDataItem(this,dataItem);
	
	/* Build the initial point rendering shader: */
	buildShader(contextData,dataItem);
	
	/* Create the primitive distance color map: */
	const int numPlaneColors=7;
	static const GLColor<GLfloat,3> planeColors[numPlaneColors]=
		{
		GLColor<GLfloat,3>(0.0f,0.0f,0.5f),
		GLColor<GLfloat,3>(0.0f,0.0f,1.0f),
		GLColor<GLfloat,3>(0.0f,1.0f,1.0f),
		GLColor<GLfloat,3>(1.0f,1.0f,1.0f),
		GLColor<GLfloat,3>(1.0f,1.0f,0.0f),
		GLColor<GLfloat,3>(1.0f,0.0f,0.0f),
		GLColor<GLfloat,3>(0.5f,0.0f,0.0f)
		};
	
	/* Create a high-resolution color map: */
	GLColor<GLfloat,3>* planeColorMap=new GLColor<GLfloat,3>[1024];
	for(int i=0;i<1024;++i)
		{
		int ci0=(i*(numPlaneColors-1))/1023;
		int ci1=ci0<numPlaneColors-1?ci0+1:numPlaneColors-1;
		float cd=float(i*(numPlaneColors-1)-ci0*1023)/1023.0f;
		for(int j=0;j<3;++j)
			planeColorMap[i][j]=planeColors[ci0][j]*(1.0f-cd)+planeColors[ci1][j]*cd;
		}
	
	#if 0
	
	/* Add notches to the color map: */
	for(int i=0;i<=20;++i)
		{
		planeColorMap[(i*1023+10)/20]=GLColor<GLfloat,3>(0.0f,0.0f,0.0f);
		}
	
	#endif
	
	/* Create the color map texture image: */
	glBindTexture(GL_TEXTURE_1D,dataItem->distMapTexture);
	glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
	glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
	glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_BASE_LEVEL,0);
	glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MAX_LEVEL,0);
	glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
	glTexImage1D(GL_TEXTURE_1D,0,GL_RGB,1024,0,GL_RGB,GL_FLOAT,planeColorMap);
	glBindTexture(GL_TEXTURE_1D,0);
	
	/* Destroy the temporary color map: */
	delete[] planeColorMap;
	}

void PointShader::setDistancePrimitive(Primitive* newDistancePrimitive)
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

void PointShader::setDistanceScale(Primitive::Scalar newDistScale)
	{
	distScale=newDistScale;
	}

void PointShader::setUseLighting(bool newUseLighting)
	{
	/* Update the lighting flag: */
	if(useLighting!=newUseLighting)
		{
		useLighting=newUseLighting;
		++settingsVersion;
		}
	}

void PointShader::setUsePointColors(bool newUsePointColors)
	{
	/* Update the point color flag: */
	if(usePointColors!=newUsePointColors)
		{
		usePointColors=newUsePointColors;
		++settingsVersion;
		}
	}

void PointShader::setUseSurfels(bool newUseSurfels)
	{
	/* Update the surfel rendering flag: */
	if(useSurfels!=newUseSurfels)
		{
		useSurfels=newUseSurfels;
		++settingsVersion;
		}
	}

void PointShader::setSurfelScale(Primitive::Scalar newSurfelScale)
	{
	surfelScale=newSurfelScale;
	}

PointShader::DataItem* PointShader::enable(GLContextData& contextData) const
	{
	/* Retrieve the context data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	try
		{
		/* Rebuild the shader if it is out of line with current state: */
		const GLLightTracker& lt=*(contextData.getLightTracker());
		const GLClipPlaneTracker& cpt=*(contextData.getClipPlaneTracker());
		if(dataItem->lightStateVersion!=lt.getVersion()||dataItem->clipPlaneStateVersion!=cpt.getVersion()||dataItem->settingsVersion!=settingsVersion)
			buildShader(contextData,dataItem);
		
		/* Enable the shader: */
		glUseProgramObjectARB(dataItem->programObject);
		
		/* Upload primitive distance calculation parameters: */
		switch(distPrimitiveType)
			{
			case DistPoint:
				{
				Geometry::Point<GLfloat,3> dc(distCenter);
				glUniformARB<3>(dataItem->distCenterLocation,1,dc.getComponents());
				glUniformARB(dataItem->distOffsetLocation,GLfloat(distOffset));
				glUniformARB(dataItem->distScaleLocation,GLfloat(distScale));
				
				break;
				}
			
			case DistLine:
				{
				Geometry::Point<GLfloat,3> dc(distCenter);
				glUniformARB<3>(dataItem->distCenterLocation,1,dc.getComponents());
				Geometry::Point<GLfloat,3> da(distAxis);
				glUniformARB<3>(dataItem->distAxisLocation,1,da.getComponents());
				glUniformARB(dataItem->distOffsetLocation,GLfloat(distOffset));
				glUniformARB(dataItem->distScaleLocation,GLfloat(distScale));
				
				break;
				}
			
			case DistPlane:
				{
				GLfloat dp[4];
				for(int i=0;i<3;++i)
					dp[i]=GLfloat(distPlane.getNormal()[i]);
				dp[3]=GLfloat(-distPlane.getOffset());
				glUniformARB<4>(dataItem->distPlaneLocation,1,dp);
				glUniformARB(dataItem->distScaleLocation,GLfloat(distScale));
				
				break;
				}
			
			default:
				;
			}
		
		if(distPrimitiveType!=DistNone)
			{
			/* Bind the distance color map texture: */
			glActiveTextureARB(GL_TEXTURE0_ARB);
			glBindTexture(GL_TEXTURE_1D,dataItem->distMapTexture);
			glUniformARB(dataItem->distMapLocation,0);
			}
		
		if(useSurfels&&dataItem->haveGeometryShaders)
			glUniformARB(dataItem->surfelScaleLocation,GLfloat(surfelScale));
		}
	catch(const std::runtime_error& err)
		{
		std::cerr<<"Disabling point rendering shader due to exception "<<err.what()<<std::endl;
		
		return 0;
		}
	
	return dataItem;
	}

void PointShader::disable(PointShader::DataItem* dataItem) const
	{
	/* Disable all shader programs: */
	glUseProgramObjectARB(0);
	
	/* Protect the distance color map texture: */
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_1D,0);
	}
