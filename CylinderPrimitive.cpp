/***********************************************************************
CylinderPrimitive - Class for cylinders extracted from point clouds.
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

#define GEOMETRY_NONSTANDARD_TEMPLATES 1

#include "CylinderPrimitive.h"

#include <stdexcept>
#include <iostream>
#include <IO/File.h>
#include <Cluster/MulticastPipe.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <Geometry/OutputOperators.h>
#include <GL/GLColorTemplates.h>
#include <GL/GLContextData.h>
#include <GL/GLGeometryWrappers.h>
#if USE_COLLABORATION
#include <Collaboration2/DataType.h>
#include <Collaboration2/DataType.icpp>
#endif

#include "LidarOctree.h"
#include "LidarSelectionExtractor.h"
#include "CylinderFitter.h"
#include "LevenbergMarquardtMinimizer.h"

/********************************************
Methods of class CylinderPrimitive::DataItem:
********************************************/

CylinderPrimitive::DataItem::DataItem(void)
	:displayListId(glGenLists(1)),
	 version(0)
	{
	}

CylinderPrimitive::DataItem::~DataItem(void)
	{
	glDeleteLists(displayListId,1);
	}

/******************************************
Static elements of class CylinderPrimitive:
******************************************/

#if USE_COLLABORATION

DataType::TypeID CylinderPrimitive::type=DataType::TypeID(-1);

#endif

/**********************************
Methods of class CylinderPrimitive:
**********************************/

CylinderPrimitive::CylinderPrimitive(const LidarOctree* octree,const Primitive::Vector& translation)
	{
	/* Extract all selected points from the octree: */
	LidarSelectionExtractor<CylinderFitter::Point> lse;
	octree->processSelectedPoints(lse);
	
	if(lse.getPoints().size()>=6)
		{
		/* Try to fit a cylinder starting with all the primary axes: */
		Scalar minF=Math::Constants<Scalar>::max;
		for(int initialAxis=0;initialAxis<3;++initialAxis)
			{
			/* Create a cylinder fitter: */
			CylinderFitter cf(lse.getPoints(),initialAxis);
			
			/* Minimize the target function: */
			Scalar f=LevenbergMarquardtMinimizer<CylinderFitter>::minimize(cf);
			
			if(minF>f)
				{
				/* Store the target function: */
				minF=f;
				
				/* Extract the cylinder parameters: */
				center=cf.getCenter();
				axis=cf.getAxis();
				axis.normalize();
				radius=cf.getRadius();
				}
			}
		
		/* Store the number of points and the RMS residual: */
		numPoints=Misc::UInt64(lse.getPoints().size());
		rms=Math::sqrt(minF*Scalar(2)/Scalar(numPoints));
		
		/* Calculate the point set's coverage along the cylinder axis: */
		std::vector<CylinderFitter::Point>::const_iterator pIt=lse.getPoints().begin();
		extents[1]=extents[0]=(*pIt-center)*axis;
		for(++pIt;pIt!=lse.getPoints().end();++pIt)
			{
			Scalar d=(*pIt-center)*axis;
			if(extents[0]>d)
				extents[0]=d;
			else if(extents[1]<d)
				extents[1]=d;
			}
		
		/* Set the cylinder's height and adjust the center point: */
		length=(extents[1]-extents[0])*Scalar(1.1);
		center+=axis*Math::mid(extents[0],extents[1]);
		extents[1]=Math::div2(length);
		extents[0]=-extents[1];
		
		/* Print the cylinder's equation: */
		std::cout<<"Cylinder fitting "<<numPoints<<" points"<<std::endl;
		std::cout<<"Center point: "<<(center+translation)<<std::endl;
		std::cout<<"Axis direction: "<<axis<<std::endl;
		std::cout<<"Radius: "<<radius<<", height: "<<length<<std::endl;
		std::cout<<"RMS approximation residual: "<<rms<<std::endl;
		
		/* Compute an appropriate number of grid lines in x and y: */
		Scalar aspect=(Scalar(2)*Math::Constants<Scalar>::pi*radius)/length;
		if(aspect>=Scalar(1))
			{
			numLines[0]=10;
			numLines[1]=int(Math::floor(Scalar(10)/aspect+Scalar(0.5)));
			}
		else
			{
			numLines[1]=10;
			numLines[0]=int(Math::floor(Scalar(10)*aspect+Scalar(0.5)));
			}
		}
	else
		throw std::runtime_error("CylinderPrimitive::CylinderPrimitive: Not enough selected points");
	}

void CylinderPrimitive::write(IO::File& file,const Vector& translation) const
	{
	/* Call the base class method: */
	LinePrimitive::write(file,translation);
	
	/* Write the radius: */
	file.write(radius);
	
	/* Write the cylinder's visual representation: */
	file.write(numLines,2);
	}

void CylinderPrimitive::read(IO::File& file,const Vector& translation)
	{
	/* Call the base class method: */
	LinePrimitive::read(file,translation);
	
	/* Read the radius: */
	file.read(radius);
	
	/* Read the cylinder's visual representation: */
	file.read(numLines,2);
	}

void CylinderPrimitive::write(Cluster::MulticastPipe* pipe) const
	{
	/* Call the base class method: */
	LinePrimitive::write(pipe);
	
	/* Write the radius: */
	pipe->write(radius);
	
	/* Write the cylinder's visual representation: */
	pipe->write(numLines,2);
	}

void CylinderPrimitive::read(Cluster::MulticastPipe* pipe)
	{
	/* Call the base class method: */
	LinePrimitive::read(pipe);
	
	/* Read the radius: */
	pipe->read(radius);
	
	/* Read the cylinder's visual representation: */
	pipe->read(numLines,2);
	}

#if USE_COLLABORATION

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"

void CylinderPrimitive::registerType(DataType& dataType)
	{
	/* Retrieve the base primitive's structure elements: */
	std::vector<DataType::StructureElement> elements=dataType.getStructureElements(LinePrimitive::type);
	
	/* Extend the base type: */
	elements.push_back(DataType::StructureElement(Primitive::scalarType,offsetof(CylinderPrimitive,radius)));
	elements.push_back(DataType::StructureElement(dataType.createFixedArray(2,DataType::getAtomicType<int>()),offsetof(CylinderPrimitive,numLines)));
	
	/* Register the extended type: */
	type=dataType.createStructure(elements,sizeof(CylinderPrimitive));
	}

#pragma GCC diagnostic pop

DataType::TypeID CylinderPrimitive::getType(void) const
	{
	return type;
	}

#endif

Primitive::DragState* CylinderPrimitive::pick(const Primitive::Point& pickPoint,Primitive::Scalar& maxPickDistance2)
	{
	/* Calculate the pick point's distance from the cylinder's axis and mantle: */
	Scalar axisDist2=Geometry::sqr(axis^(pickPoint-center));
	Scalar mantleDist2=Math::sqr(Math::sqrt(axisDist2)-radius);
	Scalar axisMantleDist2=Math::min(axisDist2,mantleDist2);
	
	/* Bail out if the pick point is neither close to the central axis nor to the mantle: */
	if(axisMantleDist2>=maxPickDistance2)
		return 0;
	
	/* Calculate the position of the pick point along the line's axis: */
	Scalar axisParam=(pickPoint-center)*axis;
	
	/* Check the pick position against the lower or upper cylinder endpoints: */
	Scalar mid=Math::mid(extents[0],extents[1]);
	if(axisParam<=mid)
		{
		/* Check the lower endpoint: */
		Scalar dist2=axisMantleDist2+Math::sqr(axisParam-extents[0]);
		if(maxPickDistance2>dist2)
			{
			/* Pick the lower endpoint: */
			maxPickDistance2=dist2;
			return new LinePrimitive::DragState(this,LinePrimitive::DragState::Lower,extents[0]-axisParam);
			}
		}
	else
		{
		/* Check the upper endpoint: */
		Scalar dist2=axisMantleDist2+Math::sqr(axisParam-extents[1]);
		if(maxPickDistance2>dist2)
			{
			/* Pick the upper endpoint: */
			maxPickDistance2=dist2;
			return new LinePrimitive::DragState(this,LinePrimitive::DragState::Upper,extents[1]-axisParam);
			}
		}
	
	/* Check the pick position against the cylinder's central axis or mantle: */
	if(axisParam>=extents[0]&&axisParam<=extents[1])
		{
		/* Pick the line: */
		maxPickDistance2=axisMantleDist2;
		return new LinePrimitive::DragState(this,LinePrimitive::DragState::Line,0);
		}
	
	/* Return no pick: */
	return 0;
	}

void CylinderPrimitive::drag(Primitive::DragState* dragState,const Point& dragPoint)
	{
	/* Call the base class method: */
	LinePrimitive::drag(dragState,dragPoint);
	
	/* Compute an appropriate number of grid lines in x and y: */
	Scalar aspect=(Scalar(2)*Math::Constants<Scalar>::pi*radius)/(extents[1]-extents[0]);
	if(aspect>=Scalar(1))
		{
		numLines[0]=10;
		numLines[1]=int(Math::floor(Scalar(10)/aspect+Scalar(0.5)));
		}
	else
		{
		numLines[1]=10;
		numLines[0]=int(Math::floor(Scalar(10)*aspect+Scalar(0.5)));
		}
	}

void CylinderPrimitive::glRenderActionTransparent(GLContextData& contextData) const
	{
	/* Retrieve the data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	/* Check if the display list is outdated: */
	if(dataItem->version!=version)
		{
		/* Regenerate the display list: */
		glNewList(dataItem->displayListId,GL_COMPILE_AND_EXECUTE);
		
		/* Create a coordinate system for the cylinder: */
		Vector cx=Geometry::normal(axis);
		cx.normalize();
		Vector cy=axis^cx;
		cy.normalize();
		Vector z0=axis*extents[0];
		Vector z1=axis*extents[1];
		
		/* Draw the cylinder's surface: */
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		glBegin(GL_QUAD_STRIP);
		glColor<4>(surfaceColor.getComponents());
		glNormal(1.0,0.0,0.0);
		glVertex(center+cx*radius+z1);
		glVertex(center+cx*radius+z0);
		for(int x=1;x<72;++x)
			{
			Scalar angle=Math::rad(Scalar(x)*Scalar(360/72));
			Vector d=cx*Math::cos(angle)+cy*Math::sin(angle);
			glNormal(d);
			d*=radius;
			glVertex(center+d+z1);
			glVertex(center+d+z0);
			}
		glNormal(1.0,0.0,0.0);
		glVertex(center+cx*radius+z1);
		glVertex(center+cx*radius+z0);
		glEnd();
		
		/* Draw the cylinder's grid: */
		glBlendFunc(GL_ONE,GL_ONE);
		glLineWidth(1.0f);
		glBegin(GL_LINES);
		glColor<4>(gridColor.getComponents());
		for(int x=0;x<numLines[0];++x)
			{
			Scalar angle=Math::rad(Scalar(x)*Scalar(360)/Scalar(numLines[0]));
			Vector d=cx*Math::cos(angle)+cy*Math::sin(angle);
			d*=radius;
			glVertex(center+d+z1);
			glVertex(center+d+z0);
			}
		glEnd();
		for(int y=0;y<=numLines[1];++y)
			{
			Point c=center+axis*(Scalar(y)*(extents[1]-extents[0])/Scalar(numLines[1])+extents[0]);
			glBegin(GL_LINE_LOOP);
			for(int x=0;x<72;++x)
				{
				Scalar angle=Math::rad(Scalar(x)*Scalar(360/72));
				Vector d=cx*Math::cos(angle)+cy*Math::sin(angle);
				d*=radius;
				glVertex(c+d);
				}
			glEnd();
			}
		
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

void CylinderPrimitive::initContext(GLContextData& contextData) const
	{
	/* Create a data item and store it in the context: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	}
