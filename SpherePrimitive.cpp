/***********************************************************************
SpherePrimitive - Class for spheres extracted from point clouds.
Copyright (c) 2007-2020 Oliver Kreylos

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

#include "SpherePrimitive.h"

#include <stdexcept>
#include <iostream>
#include <IO/File.h>
#include <Cluster/MulticastPipe.h>
#include <Math/Math.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/OutputOperators.h>
#include <GL/GLColorTemplates.h>
#include <GL/GLContextData.h>
#include <GL/GLModels.h>
#include <GL/GLGeometryWrappers.h>
#if USE_COLLABORATION
#include <Collaboration2/DataType.h>
#endif

#include "LidarOctree.h"
#include "LidarSelectionExtractor.h"
#include "SphereFitter.h"
#include "LevenbergMarquardtMinimizer.h"

/******************************************
Methods of class SpherePrimitive::DataItem:
******************************************/

SpherePrimitive::DataItem::DataItem(void)
	:displayListId(glGenLists(1)),
	 version(0)
	{
	}

SpherePrimitive::DataItem::~DataItem(void)
	{
	glDeleteLists(displayListId,1);
	}

/****************************************
Static elements of class SpherePrimitive:
****************************************/

#if USE_COLLABORATION

DataType::TypeID SpherePrimitive::type=DataType::TypeID(-1);

#endif

/********************************
Methods of class SpherePrimitive:
********************************/

SpherePrimitive::SpherePrimitive(const LidarOctree* octree,const Primitive::Vector& translation)
	{
	/* Extract all selected points from the octree: */
	LidarSelectionExtractor<SphereFitter::Point> lse;
	octree->processSelectedPoints(lse);
	
	if(lse.getPoints().size()>=4)
		{
		/* Create a sphere fitter: */
		SphereFitter sf(lse.getPoints());
		
		/* Minimize the target function: */
		Scalar f=LevenbergMarquardtMinimizer<SphereFitter>::minimize(sf);
		
		/* Store the number of points and the RMS residual: */
		numPoints=Misc::UInt64(lse.getPoints().size());
		rms=Math::sqrt(f*Scalar(2)/Scalar(numPoints));
		
		/* Extract the sphere parameters: */
		point=sf.getCenter();
		radius=sf.getRadius();
		
		/* Print the sphere's equation: */
		std::cout<<"Sphere fitting "<<numPoints<<" points"<<std::endl;
		std::cout<<"Center point: "<<(point+translation)<<std::endl;
		std::cout<<"Radius: "<<radius<<std::endl;
		std::cout<<"RMS approximation residual: "<<rms<<std::endl;
		}
	else
		throw std::runtime_error("SpherePrimitive::SpherePrimitive: Not enough selected points");
	}

void SpherePrimitive::write(IO::File& file,const Vector& translation) const
	{
	/* Call the base class method: */
	PointPrimitive::write(file,translation);
	
	/* Write the radius: */
	file.write(radius);
	}

void SpherePrimitive::read(IO::File& file,const Vector& translation)
	{
	/* Call the base class method: */
	PointPrimitive::read(file,translation);
	
	/* Read the radius: */
	file.read(radius);
	}

void SpherePrimitive::write(Cluster::MulticastPipe* pipe) const
	{
	/* Call the base class method: */
	PointPrimitive::write(pipe);
	
	/* Write the radius: */
	pipe->write(radius);
	}

void SpherePrimitive::read(Cluster::MulticastPipe* pipe)
	{
	/* Call the base class method: */
	PointPrimitive::read(pipe);
	
	/* Read the radius: */
	pipe->read(radius);
	}

#if USE_COLLABORATION

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"

void SpherePrimitive::registerType(DataType& dataType)
	{
	/* Retrieve the base primitive's structure elements: */
	std::vector<DataType::StructureElement> elements=dataType.getStructureElements(PointPrimitive::type);
	
	/* Extend the base type: */
	elements.push_back(DataType::StructureElement(Primitive::scalarType,offsetof(SpherePrimitive,radius)));
	
	/* Register the extended type: */
	type=dataType.createStructure(elements,sizeof(SpherePrimitive));
	}

#pragma GCC diagnostic pop

DataType::TypeID SpherePrimitive::getType(void) const
	{
	return type;
	}

#endif

Primitive::DragState* SpherePrimitive::pick(const Primitive::Point& pickPoint,Primitive::Scalar& maxPickDistance2)
	{
	/* Calculate the pick point's distance from the sphere's center or its surface: */
	Scalar centerDist2=Geometry::sqrDist(pickPoint,point);
	Scalar surfaceDist2=Math::sqr(Math::sqrt(centerDist2)-radius);
	Scalar centerSurfaceDist2=Math::min(centerDist2,surfaceDist2);
	
	/* Pick the sphere if the distance from the pick point to the center or surface is smaller than the maximum distance: */
	if(maxPickDistance2>centerSurfaceDist2)
		{
		/* Update the maximum pick distance and return a new drag state: */
		maxPickDistance2=centerSurfaceDist2;
		return new Primitive::DragState(this);
		}
	else
		return 0;
	}

void SpherePrimitive::glRenderAction(GLContextData& contextData) const
	{
	/* Draw the sphere's center: */
	glPointSize(3.0f);
	glBegin(GL_POINTS);
	glColor<4>(surfaceColor.getComponents());
	glVertex(point);
	glEnd();
	}

void SpherePrimitive::glRenderActionTransparent(GLContextData& contextData) const
	{
	/* Retrieve the data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	/* Check if the display list is outdated: */
	if(dataItem->version!=version)
		{
		/* Regenerate the display list: */
		glNewList(dataItem->displayListId,GL_COMPILE_AND_EXECUTE);
		
		glPushMatrix();
		glTranslate(point-Point::origin);
		
		/* Draw the sphere's surface: */
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		glColor<4>(surfaceColor.getComponents());
		glDrawSphereIcosahedron(radius,5);
		
		/* Draw the sphere's grid: */
		glBlendFunc(GL_ONE,GL_ONE);
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
		glLineWidth(1.0f);
		glColor<4>(gridColor.getComponents());
		glDrawSphereIcosahedron(radius,5);
		
		glPopMatrix();
		
		glEndList();
		
		/* Mark the display list as up-to-date: */
		dataItem->version=version;
		}
	else
		{
		/* Execute the display list: */
		glCallList(dataItem->displayListId);
		}
	}

void SpherePrimitive::initContext(GLContextData& contextData) const
	{
	/* Create a data item and store it in the context: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	}
