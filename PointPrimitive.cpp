/***********************************************************************
PointPrimitive - Class for points extracted from point clouds by
intersecting three plane primitives or one line primitive and one plane
primitive.
Copyright (c) 2008-2020 Oliver Kreylos

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

#include "PointPrimitive.h"

#include <stdexcept>
#include <iostream>
#include <IO/File.h>
#include <Cluster/MulticastPipe.h>
#include <Math/Math.h>
#include <Math/Matrix.h>
#include <Geometry/AffineCombiner.h>
#include <Geometry/Vector.h>
#include <Geometry/OutputOperators.h>
#include <GL/gl.h>
#include <GL/GLColorTemplates.h>
#include <GL/GLGeometryWrappers.h>
#if USE_COLLABORATION
#include <Collaboration2/DataType.h>
#endif

#include "LinePrimitive.h"
#include "PlanePrimitive.h"

/***************************************
Static elements of class PointPrimitive:
***************************************/

#if USE_COLLABORATION

DataType::TypeID PointPrimitive::type=DataType::TypeID(-1);

#endif

/*******************************
Methods of class PointPrimitive:
*******************************/

PointPrimitive::PointPrimitive(const PlanePrimitive* const ps[3],const Primitive::Vector& translation)
	{
	/* Calculate the centroid of the three planes' center points for conditioning: */
	Geometry::AffineCombiner<Point::Scalar,Point::dimension> cc;
	for(int i=0;i<3;++i)
		cc.addPoint(ps[i]->getCenter());
	Point centroid=cc.getPoint();
	
	/* Create a linear system to intersect the three planes: */
	Math::Matrix a(3,3);
	Math::Matrix b(3,1);
	for(int i=0;i<3;++i)
		{
		for(int j=0;j<3;++j)
			a(i,j)=double(ps[i]->getNormal()[j]);
		b(i)=double((ps[i]->getCenter()-centroid)*ps[i]->getNormal());
		}
	
	/* Solve the linear system: */
	Math::Matrix p=b.divideFullPivot(a);
	
	/* Extract and un-condition the result: */
	point=centroid;
	point+=Vector(Scalar(p(0)),Scalar(p(1)),Scalar(p(2)));
	
	/* Calculate the result's RMS from the source planes' RMSs: */
	numPoints=0;
	rms=Scalar(0);
	for(int i=0;i<3;++i)
		{
		numPoints+=ps[i]->getNumPoints();
		rms+=Math::sqr(ps[i]->getRms())*Scalar(ps[i]->getNumPoints());
		}
	rms=Math::sqrt(rms/Scalar(numPoints));
	
	/* Print the point's equation: */
	std::cout<<"Point intersecting three planes, based on "<<numPoints<<" points"<<std::endl;
	std::cout<<"Point: "<<(point+translation)<<std::endl;
	std::cout<<"RMS approximation residual: "<<rms<<std::endl;
	}

PointPrimitive::PointPrimitive(const PlanePrimitive* p,const LinePrimitive* l,const Primitive::Vector& translation)
	{
	/* Get the plane's plane equation: */
	const Point& pc=p->getCenter();
	const Vector& pn=p->getNormal();
	
	/* Get the line's line equation: */
	const Point& lc=l->getCenter();
	const Vector& la=l->getAxis();
	
	/* Intersect the plane and the line: */
	Scalar denominator=la*pn;
	if(denominator!=Scalar(0))
		{
		/* Calculate the intersection point: */
		Scalar lambda=((pc-lc)*pn)/denominator;
		point=lc+la*lambda;
		
		/* Calculate the result's RMS from the source primitives' RMSs: */
		numPoints=p->getNumPoints()+l->getNumPoints();
		rms=Math::sqr(p->getRms())*Scalar(p->getNumPoints())+Math::sqr(l->getRms())*Scalar(l->getNumPoints());
		rms=Math::sqrt(rms/Scalar(numPoints));
		
		/* Print the point's equation: */
		std::cout<<"Point intersecting one plane and one line, based on "<<numPoints<<" points"<<std::endl;
		std::cout<<"Point: "<<(point+translation)<<std::endl;
		std::cout<<"RMS approximation residual: "<<rms<<std::endl;
		}
	else
		throw std::runtime_error("PointPrimitive::PointPrimitive: Plane and line do not intersect");
	}

void PointPrimitive::write(IO::File& file,const Vector& translation) const
	{
	/* Call the base class method: */
	Primitive::write(file,translation);
	
	/* Write the translated point: */
	file.write((point+translation).getComponents(),3);
	}

void PointPrimitive::read(IO::File& file,const Vector& translation)
	{
	/* Call the base class method: */
	Primitive::read(file,translation);
	
	/* Read the point and apply the translation: */
	file.read(point.getComponents(),3);
	point+=translation;
	}

void PointPrimitive::write(Cluster::MulticastPipe* pipe) const
	{
	/* Call the base class method: */
	Primitive::write(pipe);
	
	/* Write the point: */
	pipe->write(point.getComponents(),3);
	}

void PointPrimitive::read(Cluster::MulticastPipe* pipe)
	{
	/* Call the base class method: */
	Primitive::read(pipe);
	
	/* Read the point: */
	pipe->read(point.getComponents(),3);
	}

#if USE_COLLABORATION

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"

void PointPrimitive::registerType(DataType& dataType)
	{
	/* Retrieve the base primitive's structure elements: */
	std::vector<DataType::StructureElement> elements=dataType.getStructureElements(Primitive::type);
	
	/* Extend the base type: */
	elements.push_back(DataType::StructureElement(Primitive::pointType,offsetof(PointPrimitive,point)));
	
	/* Register the extended type: */
	type=dataType.createStructure(elements,sizeof(PointPrimitive));
	}

#pragma GCC diagnostic pop

DataType::TypeID PointPrimitive::getType(void) const
	{
	return type;
	}

#endif

Primitive::DragState* PointPrimitive::pick(const Primitive::Point& pickPoint,Primitive::Scalar& maxPickDistance2)
	{
	/* Pick if the distance from the pick point to the point is smaller than the maximum distance: */
	Scalar dist2=Geometry::sqrDist(pickPoint,point);
	if(maxPickDistance2>dist2)
		{
		/* Update the maximum pick distance and return a new drag state: */
		maxPickDistance2=dist2;
		return new Primitive::DragState(this);
		}
	else
		return 0;
	}

void PointPrimitive::glRenderAction(GLContextData& contextData) const
	{
	/* Draw the point: */
	glPointSize(3.0f);
	glBegin(GL_POINTS);
	glColor<4>(surfaceColor.getComponents());
	glVertex(point);
	glEnd();
	}
