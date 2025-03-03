/***********************************************************************
LinePrimitive - Class for lines extracted from point clouds by
intersecting two plane primitives.
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

#include "LinePrimitive.h"

#include <stdexcept>
#include <iostream>
#include <IO/File.h>
#include <Cluster/MulticastPipe.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <Math/Matrix.h>
#include <Geometry/AffineCombiner.h>
#include <Geometry/Box.h>
#include <Geometry/PCACalculator.h>
#include <Geometry/OutputOperators.h>
#include <GL/gl.h>
#include <GL/GLColorTemplates.h>
#include <GL/GLGeometryWrappers.h>
#if USE_COLLABORATION
#include <Collaboration2/DataType.h>
#endif

#include "LidarOctree.h"
#include "PlanePrimitive.h"

/**************************************
Static elements of class LinePrimitive:
**************************************/

#if USE_COLLABORATION

DataType::TypeID LinePrimitive::type=DataType::TypeID(-1);

#endif

/******************************
Methods of class LinePrimitive:
******************************/

namespace {

/**************
Helper classes:
**************/

class LidarLineExtractor
	{
	/* Embedded classes: */
	public:
	typedef Geometry::Point<double,3> Point; // Type for points
	typedef Geometry::Vector<double,3> Vector; // Type for vectors
	typedef Geometry::Box<double,3> Box; // Type for bounding boxes
	
	/* Elements: */
	private:
	Box bb; // Bounding box of all processed points
	Geometry::PCACalculator<3> pca; // Helper object to accumulate the points' covariance matrix and calculate their PCA
	
	/* Constructors and destructors: */
	public:
	LidarLineExtractor(void)
		:bb(Box::empty)
		{
		};
	
	/* Methods: */
	void operator()(const LidarPoint& lp) // Process the given LiDAR point
		{
		/* Add the node point to the bounding box: */
		bb.addPoint(lp);
		
		/* Add the point to the PCA calculator: */
		pca.accumulatePoint(lp);
		};
	size_t getNumPoints(void) const // Returns the number of processed points
		{
		return pca.getNumPoints();
		}
	const Box& getBB(void) const // Returns the processed points' bounding box
		{
		return bb;
		};
	void calcLine(Point& centroid,Vector& axis) // Returns the least-squares line
		{
		/* Calculate the processed points' centroid: */
		centroid=pca.calcCentroid();
		
		/* Calculate the point set's covariance matrix: */
		pca.calcCovariance();
		
		/* Calculate the covariance matrix' eigenvalues: */
		double evs[3];
		pca.calcEigenvalues(evs);
		
		/* Get the "longest" eigenvector: */
		axis=pca.calcEigenvector(evs[0]);
		};
	};

class LidarLineFitter
	{
	/* Embedded classes: */
	public:
	typedef Geometry::Point<double,3> Point; // Type for points
	typedef Geometry::Vector<double,3> Vector; // Type for vectors
	
	/* Elements: */
	private:
	Point centroid; // Line's centroid
	Vector axis; // Line's normalized axis
	double min,max; // Bounding interval of all points in line's coordinate system
	size_t numPoints; // Number of accumulated points
	double ms; // Accumulated RMS distance from points to line
	
	/* Constructors and destructors: */
	public:
	LidarLineFitter(const Point& sCentroid,const Vector& sAxis)
		:centroid(sCentroid),axis(sAxis),
		 min(Math::Constants<double>::max),
		 max(Math::Constants<double>::min),
		 numPoints(0),ms(0.0)
		{
		/* Normalize the axis vector: */
		axis.normalize();
		};
	
	/* Methods: */
	void operator()(const LidarPoint& lp) // Process the given LiDAR point
		{
		/* Transform the point to line coordinates: */
		Vector lpc=Point(lp)-centroid;
		double x=lpc*axis;
		
		/* Add the point to the bounding interval: */
		if(min>x)
			min=x;
		if(max<x)
			max=x;
		
		/* Add the point to the RMS distance: */
		++numPoints;
		ms+=Geometry::sqr(lpc-axis*x);
		};
	double getMin(void) const
		{
		return min;
		};
	double getMax(void) const
		{
		return max;
		};
	double getRMS(void) const
		{
		return Math::sqrt(ms/double(numPoints));
		};
	};

}

LinePrimitive::LinePrimitive(const LidarOctree* octree,const Vector& translation)
	{
	/* Create a LiDAR line extractor: */
	LidarLineExtractor lle;
	
	/* Process all selected points: */
	octree->processSelectedPoints(lle);
	
	if(lle.getNumPoints()>=2)
		{
		/* Extract the line's coordinate frame: */
		LidarLineExtractor::Point centroid;
		LidarLineExtractor::Vector laxis;
		lle.calcLine(centroid,laxis);
		
		/* Calculate the bounding interval of the selected points in line coordinates: */
		LidarLineFitter llf(centroid,laxis);
		octree->processSelectedPoints(llf);
		
		/* Store the number of points and the RMS residual: */
		numPoints=Misc::UInt64(lle.getNumPoints());
		rms=Scalar(llf.getRMS());
		
		/* Initialize the extracted line primitive's extents: */
		extents=LinePrimitive::Interval(Scalar(llf.getMin()),Scalar(llf.getMax()));
		length=(extents[1]-extents[0])*Scalar(1.1);
		
		/* Shift the line's center to the middle of its extent interval: */
		laxis.normalize();
		center=Point(centroid+laxis*Math::mid(extents[0],extents[1]));
		axis=Vector(laxis);
		extents[1]=Math::div2(length);
		extents[0]=-extents[1];
		
		/* Print the line's equation: */
		std::cout<<"Line fitting "<<numPoints<<" points"<<std::endl;
		std::cout<<"Center point: "<<(center+translation)<<std::endl;
		std::cout<<"Axis direction: "<<axis<<std::endl;
		std::cout<<"Length: "<<length<<std::endl;
		std::cout<<"RMS approximation residual: "<<rms<<std::endl;
		}
	else
		throw std::runtime_error("LinePrimitive::LinePrimitive: Not enough selected points");
	}

LinePrimitive::LinePrimitive(const PlanePrimitive* const ps[2],const Primitive::Vector& translation)
	{
	/* Calculate the centroid of the two planes' center points for conditioning: */
	Geometry::AffineCombiner<Point::Scalar,Point::dimension> cc;
	for(int i=0;i<2;++i)
		cc.addPoint(ps[i]->getCenter());
	Point centroid=cc.getPoint();
	
	/* Create an underdetermined linear system to intersect the two planes: */
	Math::Matrix a(3,3,0.0);
	Math::Matrix b(3,1,0.0);
	for(int i=0;i<2;++i)
		{
		for(int j=0;j<3;++j)
			a(i,j)=double(ps[i]->getNormal()[j]);
		b(i)=double((ps[i]->getCenter()-centroid)*ps[i]->getNormal());
		}
	
	/* Find the linear system's null space: */
	std::pair<Math::Matrix,Math::Matrix> sol=a.solveLinearSystem(b);
	
	/* Check if there is a well-defined solution: */
	if(sol.first.getNumColumns()==1&&sol.second.getNumColumns()==1)
		{
		/* Calculate the result's RMS from the source planes' RMSs: */
		numPoints=0;
		rms=Scalar(0);
		for(int i=0;i<2;++i)
			{
			numPoints+=ps[i]->getNumPoints();
			rms+=Math::sqr(ps[i]->getRms())*Scalar(ps[i]->getNumPoints());
			}
		rms=Math::sqrt(rms/Scalar(numPoints));
		
		/* Extract and un-condition the result: */
		center=centroid;
		center+=Vector(Scalar(sol.first(0)),Scalar(sol.first(1)),Scalar(sol.first(2)));
		axis=Vector(Scalar(sol.second(0)),Scalar(sol.second(1)),Scalar(sol.second(2)));
		axis.normalize();
		
		/* Find the extents of both planes' rectangles on the line: */
		extents=LinePrimitive::Interval(Math::Constants<double>::max,Math::Constants<double>::min);
		for(int plane=0;plane<2;++plane)
			for(int i=0;i<4;++i)
				{
				double param=(ps[plane]->getCorner(i)-center)*axis;
				if(extents[0]>param)
					extents[0]=param;
				if(extents[1]<param)
					extents[1]=param;
				}
		length=(extents[1]-extents[0])*Scalar(1.1);
		
		/* Shift the line's center to the middle of its extent interval: */
		center+=axis*Math::mid(extents[0],extents[1]);
		extents[1]=Math::div2(length);
		extents[0]=-extents[1];
		
		/* Print the line's equation: */
		std::cout<<"Line intersecting two planes, based on "<<numPoints<<" points"<<std::endl;
		std::cout<<"Center point: "<<(center+translation)<<std::endl;
		std::cout<<"Axis direction: "<<axis<<std::endl;
		std::cout<<"Length: "<<length<<std::endl;
		std::cout<<"RMS approximation residual: "<<rms<<std::endl;
		}
	else
		throw std::runtime_error("LinePrimitive::LinePrimitive: Given planes do not intersect");
	}

void LinePrimitive::write(IO::File& file,const Vector& translation) const
	{
	/* Call the base class method: */
	Primitive::write(file,translation);
	
	/* Write the translated center point: */
	file.write((center+translation).getComponents(),3);
	
	/* Write the axis direction and length: */
	file.write(axis.getComponents(),3);
	file.write(length);
	
	/* Write the extents of the line's visual representation: */
	file.write(extents.getComponents(),2);
	}

void LinePrimitive::read(IO::File& file,const Vector& translation)
	{
	/* Call the base class method: */
	Primitive::read(file,translation);
	
	/* Read the center point and apply the translation: */
	file.read(center.getComponents(),3);
	center+=translation;
	
	/* Read the axis direction and length: */
	file.read(axis.getComponents(),3);
	file.read(length);
	
	/* Read the extents of the line's visual representation: */
	file.read(extents.getComponents(),2);
	}

void LinePrimitive::write(Cluster::MulticastPipe* pipe) const
	{
	/* Call the base class method: */
	Primitive::write(pipe);
	
	/* Write the center point: */
	pipe->write(center.getComponents(),3);
	
	/* Write the axis direction and length: */
	pipe->write(axis.getComponents(),3);
	pipe->write(length);
	
	/* Write the extents of the line's visual representation: */
	pipe->write(extents.getComponents(),2);
	}

void LinePrimitive::read(Cluster::MulticastPipe* pipe)
	{
	/* Call the base class method: */
	Primitive::read(pipe);
	
	/* Read the center point: */
	pipe->read(center.getComponents(),3);
	
	/* Read the axis direction and length: */
	pipe->read(axis.getComponents(),3);
	pipe->read(length);
	
	/* Read the extents of the line's visual representation: */
	pipe->read(extents.getComponents(),2);
	}

#if USE_COLLABORATION

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"

void LinePrimitive::registerType(DataType& dataType)
	{
	/* Retrieve the base primitive's structure elements: */
	std::vector<DataType::StructureElement> elements=dataType.getStructureElements(Primitive::type);
	
	/* Extend the base type: */
	elements.push_back(DataType::StructureElement(Primitive::pointType,offsetof(LinePrimitive,center)));
	elements.push_back(DataType::StructureElement(Primitive::vectorType,offsetof(LinePrimitive,axis)));
	elements.push_back(DataType::StructureElement(Primitive::scalarType,offsetof(LinePrimitive,length)));
	elements.push_back(DataType::StructureElement(dataType.createFixedArray(2,Primitive::scalarType),offsetof(LinePrimitive,extents)));
	
	/* Register the extended type: */
	type=dataType.createStructure(elements,sizeof(LinePrimitive));
	}

#pragma GCC diagnostic pop

DataType::TypeID LinePrimitive::getType(void) const
	{
	return type;
	}

#endif

Primitive::DragState* LinePrimitive::pick(const Primitive::Point& pickPoint,Primitive::Scalar& maxPickDistance2)
	{
	/* Calculate the pick point's distance from the line's axis and bail out if it is too large: */
	Scalar axisDist2=Geometry::sqr(axis^(pickPoint-center));
	if(axisDist2>=maxPickDistance2)
		return 0;
	
	/* Calculate the position of the pick point along the line's axis: */
	Scalar axisParam=(pickPoint-center)*axis;
	
	/* Check the pick position against the lower or upper line endpoints: */
	Scalar mid=Math::mid(extents[0],extents[1]);
	if(axisParam<=mid)
		{
		/* Check the lower endpoint: */
		Scalar dist2=axisDist2+Math::sqr(axisParam-extents[0]);
		if(maxPickDistance2>dist2)
			{
			/* Pick the lower endpoint: */
			maxPickDistance2=dist2;
			return new DragState(this,DragState::Lower,extents[0]-axisParam);
			}
		}
	else
		{
		/* Check the upper endpoint: */
		Scalar dist2=axisDist2+Math::sqr(axisParam-extents[1]);
		if(maxPickDistance2>dist2)
			{
			/* Pick the upper endpoint: */
			maxPickDistance2=dist2;
			return new DragState(this,DragState::Upper,extents[1]-axisParam);
			}
		}
	
	/* Check the pick position against the line: */
	if(axisParam>=extents[0]&&axisParam<=extents[1])
		{
		/* Pick the line: */
		maxPickDistance2=axisDist2;
		return new DragState(this,DragState::Line,0);
		}
	
	/* Return no pick: */
	return 0;
	}

void LinePrimitive::drag(Primitive::DragState* dragState,const Point& dragPoint)
	{
	/* Access the real drag state: */
	DragState* ds=static_cast<DragState*>(dragState);
	
	/* Drag: */
	switch(ds->pickedPart)
		{
		case DragState::Lower:
			{
			/* Calculate the position of the drag point along the line's axis: */
			Scalar axisParam=(dragPoint-center)*axis;
			
			/* Update the line's lower extent: */
			extents[0]=axisParam+ds->pickOffset;
			
			/* Check if the upper and lower extents switched places: */
			if(extents[0]>extents[1])
				{
				/* Start dragging the upper extent: */
				std::swap(extents[0],extents[1]);
				ds->pickedPart=DragState::Upper;
				}
			
			++version;
			
			break;
			}
		
		case DragState::Upper:
			{
			/* Calculate the position of the drag point along the line's axis: */
			Scalar axisParam=(dragPoint-center)*axis;
			
			/* Update the line's upper extent: */
			extents[1]=axisParam+ds->pickOffset;
			
			/* Check if the upper and lower extents switched places: */
			if(extents[0]>extents[1])
				{
				/* Start dragging the upper extent: */
				std::swap(extents[0],extents[1]);
				ds->pickedPart=DragState::Lower;
				}
			
			++version;
			
			break;
			}
		
		default:
			; // Do nothing
		}
	}

void LinePrimitive::glRenderAction(GLContextData& contextData) const
	{
	/* Draw the line: */
	glLineWidth(3.0f);
	glBegin(GL_LINES);
	glColor<4>(surfaceColor.getComponents());
	glVertex(center+axis*extents[0]);
	glVertex(center+axis*extents[1]);
	glEnd();
	}
