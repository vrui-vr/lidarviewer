/***********************************************************************
PlanePrimitive - Class for planes extracted from point clouds.
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

#include "PlanePrimitive.h"

#include <stdexcept>
#include <iostream>
#include <IO/File.h>
#include <Cluster/MulticastPipe.h>
#include <Math/Math.h>
#include <Geometry/OutputOperators.h>
#include <GL/GLColorTemplates.h>
#include <GL/GLContextData.h>
#include <GL/GLGeometryWrappers.h>
#if USE_COLLABORATION
#include <Collaboration2/DataType.h>
#include <Collaboration2/DataType.icpp>
#endif

#include "LidarOctree.h"
#include "LidarPlaneExtractor.h"
#include "LidarPlaneFitter.h"

/*****************************************
Methods of class PlanePrimitive::DataItem:
*****************************************/

PlanePrimitive::DataItem::DataItem(void)
	:displayListId(glGenLists(1)),
	 version(0)
	{
	}

PlanePrimitive::DataItem::~DataItem(void)
	{
	glDeleteLists(displayListId,1);
	}

/***************************************
Static elements of class PlanePrimitive:
***************************************/

#if USE_COLLABORATION

DataType::TypeID PlanePrimitive::type=DataType::TypeID(-1);

#endif

/*******************************
Methods of class PlanePrimitive:
*******************************/

PlanePrimitive::PlanePrimitive(const LidarOctree* octree,const Primitive::Vector& translation)
	{
	/* Create a LiDAR plane extractor: */
	LidarPlaneExtractor lpe;
	
	/* Process all selected points: */
	octree->processSelectedPoints(lpe);
	
	if(lpe.getNumPoints()>=3)
		{
		/* Extract the plane's coordinate frame: */
		LidarPlaneExtractor::Point centroid;
		LidarPlaneExtractor::Vector planeFrame[3];
		double lengths[3];
		lpe.calcPlane(centroid,planeFrame,lengths);
		
		/* Ensure that (planeFrame, planeNormal) is a right-handed system, and that planeNormal points "up:" */
		if(planeFrame[2][2]<0.0)
			planeFrame[2]=-planeFrame[2];
		if((planeFrame[1]^planeFrame[2])*planeFrame[0]<0.0)
			planeFrame[0]=-planeFrame[0];
		
		/* Store the plane equation: */
		center=Point(centroid);
		normal=Vector(planeFrame[2]);
		normal.normalize();
		
		/* Calculate the bounding box of the selected points in plane coordinates: */
		LidarPlaneFitter lpf(centroid,planeFrame);
		octree->processSelectedPoints(lpf);
		
		/* Store the number of points and the RMS residual: */
		numPoints=Misc::UInt64(lpe.getNumPoints());
		rms=Scalar(lpf.getRMS());
		
		/* Calculate the extracted plane's rectangle: */
		for(int i=0;i<2;++i)
			axes[i]=Vector(planeFrame[i]);
		for(int i=0;i<2;++i)
			extents[i]=PlanePrimitive::Interval(Scalar(lpf.getMin(i)),lpf.getMax(i));
		Scalar size=Math::max(extents[0][1]-extents[0][0],extents[1][1]-extents[1][0]);
		for(int i=0;i<2;++i)
			{
			extents[i][0]-=size*Scalar(0.1);
			extents[i][1]+=size*Scalar(0.1);
			}
		
		/* Compute an appropriate number of grid lines in x and y: */
		Scalar aspect=(extents[0][1]-extents[0][0])/(extents[1][1]-extents[1][0]);
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
		
		/* Print the plane equation: */
		std::cout<<"Plane fitting "<<numPoints<<" points"<<std::endl;
		std::cout<<"Center: "<<(center+translation)<<std::endl;
		std::cout<<"Normal vector: "<<normal<<std::endl;
		std::cout<<"RMS approximation residual: "<<rms<<std::endl;
		}
	else
		throw std::runtime_error("PlanePrimitive::PlanePrimitive: Not enough selected points");
	}

void PlanePrimitive::write(IO::File& file,const Vector& translation) const
	{
	/* Call the base class method: */
	Primitive::write(file,translation);
	
	/* Write the translated center point and the normal vector: */
	file.write((center+translation).getComponents(),3);
	file.write(normal.getComponents(),3);
	
	/* Write the plane's visual representation: */
	for(int i=0;i<2;++i)
		file.write(axes[i].getComponents(),3);
	for(int i=0;i<2;++i)
		file.write(extents[i].getComponents(),2);
	file.write(numLines,2);
	}

void PlanePrimitive::read(IO::File& file,const Vector& translation)
	{
	/* Call the base class method: */
	Primitive::read(file,translation);
	
	/* Read the center point and the normal vector and apply the translation: */
	file.read(center.getComponents(),3);
	center+=translation;
	file.read(normal.getComponents(),3);
	
	/* Read the plane's visual representation: */
	for(int i=0;i<2;++i)
		file.read(axes[i].getComponents(),3);
	for(int i=0;i<2;++i)
		file.read(extents[i].getComponents(),2);
	file.read(numLines,2);
	}

void PlanePrimitive::write(Cluster::MulticastPipe* pipe) const
	{
	/* Call the base class method: */
	Primitive::write(pipe);
	
	/* Write the center point and the normal vector: */
	pipe->write(center.getComponents(),3);
	pipe->write(normal.getComponents(),3);
	
	/* Write the plane's visual representation: */
	for(int i=0;i<2;++i)
		pipe->write(axes[i].getComponents(),3);
	for(int i=0;i<2;++i)
		pipe->write(extents[i].getComponents(),2);
	pipe->write(numLines,2);
	}

void PlanePrimitive::read(Cluster::MulticastPipe* pipe)
	{
	/* Call the base class method: */
	Primitive::read(pipe);
	
	/* Read the center point and the normal vector: */
	pipe->read(center.getComponents(),3);
	pipe->read(normal.getComponents(),3);
	
	/* Read the plane's visual representation: */
	for(int i=0;i<2;++i)
		pipe->read(axes[i].getComponents(),3);
	for(int i=0;i<2;++i)
		pipe->read(extents[i].getComponents(),2);
	pipe->read(numLines,2);
	}

#if USE_COLLABORATION

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"

void PlanePrimitive::registerType(DataType& dataType)
	{
	/* Retrieve the base primitive's structure elements: */
	std::vector<DataType::StructureElement> elements=dataType.getStructureElements(Primitive::type);
	
	/* Extend the base type: */
	elements.push_back(DataType::StructureElement(Primitive::pointType,offsetof(PlanePrimitive,center)));
	elements.push_back(DataType::StructureElement(Primitive::vectorType,offsetof(PlanePrimitive,normal)));
	elements.push_back(DataType::StructureElement(dataType.createFixedArray(2,Primitive::vectorType),offsetof(PlanePrimitive,axes)));
	elements.push_back(DataType::StructureElement(dataType.createFixedArray(2,dataType.createFixedArray(2,Primitive::scalarType)),offsetof(PlanePrimitive,extents)));
	elements.push_back(DataType::StructureElement(dataType.createFixedArray(2,DataType::getAtomicType<int>()),offsetof(PlanePrimitive,numLines)));
	
	/* Register the extended type: */
	type=dataType.createStructure(elements,sizeof(PlanePrimitive));
	}

#pragma GCC diagnostic pop

DataType::TypeID PlanePrimitive::getType(void) const
	{
	return type;
	}

#endif

Primitive::DragState* PlanePrimitive::pick(const Primitive::Point& pickPoint,Primitive::Scalar& maxPickDistance2)
	{
	/* Calculate the pick point's distance from the plane and bail out if it is too far away: */
	Vector ppc=pickPoint-center;
	Scalar planeDist2=Math::sqr(ppc*normal);
	if(planeDist2>=maxPickDistance2)
		return 0;
	
	/* Calculate the pick point's local coordinates: */
	Scalar lpp[2];
	for(int i=0;i<2;++i)
		lpp[i]=ppc*axes[i];
	
	/* Check if the pick point is within the bounds of the plane primitive's rectangle and bail out if it is not: */
	int cornerIndex=0x0;
	Scalar cornerDist2=planeDist2;
	Scalar edgeDist2=Math::Constants<Scalar>::max;
	int edgeIndex=-1;
	Scalar faceDist2=planeDist2;
	for(int i=0;i<2;++i)
		{
		if(lpp[i]>=Math::mid(extents[i][0],extents[i][1]))
			{
			Scalar d2=Math::sqr(lpp[i]-extents[i][1]);
			cornerIndex|=0x1<<i;
			cornerDist2+=d2;
			if(edgeDist2>planeDist2+d2)
				{
				edgeDist2=planeDist2+d2;
				edgeIndex=2*i+1;
				}
			if(lpp[i]>extents[i][1])
				faceDist2+=d2;
			}
		else
			{
			Scalar d2=Math::sqr(lpp[i]-extents[i][0]);
			cornerDist2+=d2;
			if(edgeDist2>planeDist2+d2)
				{
				edgeDist2=planeDist2+d2;
				edgeIndex=2*i+0;
				}
			if(lpp[i]<extents[i][0])
				faceDist2+=d2;
			}
		}
	if(faceDist2>=maxPickDistance2)
		return 0;
	
	/* Check which part of the plane primitive was picked: */
	if(maxPickDistance2>cornerDist2)
		{
		/* Pick a corner: */
		maxPickDistance2=cornerDist2;
		return new DragState(this,DragState::Corner,cornerIndex,getCorner(cornerIndex)-pickPoint);
		}
	else if(maxPickDistance2>edgeDist2)
		{
		/* Pick an edge: */
		maxPickDistance2=edgeDist2;
		Point edgePos=center;
		int ai=edgeIndex/2;
		edgePos+=axes[ai]*((edgeIndex&0x1)?extents[ai][1]:extents[ai][0]);
		edgePos+=axes[1-ai]*Math::mid(extents[1-ai][0],extents[1-ai][1]);
		return new DragState(this,DragState::Edge,edgeIndex,edgePos-pickPoint);
		}
	else
		{
		/* Pick the face: */
		maxPickDistance2=faceDist2;
		return new DragState(this,DragState::Face,-1,Vector::zero);
		}
	}

void PlanePrimitive::drag(Primitive::DragState* dragState,const Point& dragPoint)
	{
	/* Access the real drag state: */
	DragState* ds=static_cast<DragState*>(dragState);
	
	/* Drag: */
	switch(ds->pickedPart)
		{
		case DragState::Corner:
			{
			/* Calculate the center point of the plane's visual representation: */
			Point visCenter=center;
			for(int i=0;i<2;++i)
				visCenter+=axes[i]*Math::mid(extents[i][0],extents[i][1]);
			
			/* Calculate the vector from the visual center to the dragging point: */
			Vector dpc=(dragPoint+ds->offset)-visCenter;
			dpc.orthogonalize(normal);
			
			/* Check if the drag position is well-defined: */
			if(dpc.sqr()!=Scalar(0))
				{
				/* Calculate the vector from the visual center to the dragged corner point: */
				Vector cc=getCorner(ds->pickedPartIndex)-visCenter;
				
				/* Calculate an incremental rotation angle: */
				Scalar cosAlpha=Math::clamp((dpc*cc)/Math::sqrt(dpc.sqr()*cc.sqr()),Scalar(-1),Scalar(1));
				Scalar sinAlpha=Math::sqrt(Scalar(1)-Math::sqr(cosAlpha));
				if((cc^dpc)*normal<Scalar(0))
					sinAlpha=-sinAlpha;
				
				/* Rotate the plane's local coordinate axes: */
				Vector xp=axes[0]*cosAlpha+axes[1]*sinAlpha;
				axes[1]=axes[1]*cosAlpha-axes[0]*sinAlpha;
				axes[0]=xp;
				for(int i=0;i<2;++i)
					axes[i].normalize();
				
				++version;
				}
			
			break;
			}
		
		case DragState::Edge:
			{
			/* Calculate the drag point's local coordinates: */
			Vector dpc=(dragPoint-center)+ds->offset;
			Scalar dp[2];
			for(int i=0;i<2;++i)
				dp[i]=dpc*axes[i];
			
			/* Adjust the plane's min or max extent: */
			int ai=ds->pickedPartIndex/2;
			if(ds->pickedPartIndex&0x1)
				extents[ai][1]=dp[ai];
			else
				extents[ai][0]=dp[ai];
			
			/* Check if the min and max extent changed places: */
			if(extents[ai][0]>extents[ai][1])
				{
				/* Start dragging the opposite edge: */
				std::swap(extents[ai][0],extents[ai][1]);
				ds->pickedPartIndex=ds->pickedPartIndex^0x1;
				}
			
			/* Compute an appropriate number of grid lines in x and y: */
			Scalar aspect=(extents[0][1]-extents[0][0])/(extents[1][1]-extents[1][0]);
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
			
			++version;
			
			break;
			}
		
		default:
			; // Do nothing
		}
	}

void PlanePrimitive::glRenderActionTransparent(GLContextData& contextData) const
	{
	/* Retrieve the data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	/* Check if the display list is outdated: */
	if(dataItem->version!=version)
		{
		/* Regenerate the display list: */
		glNewList(dataItem->displayListId,GL_COMPILE_AND_EXECUTE);
		
		/* Calculate the plane's corner points: */
		Point points[4];
		for(int i=0;i<4;++i)
			points[i]=getCorner(i);
		
		/* Draw the plane's surface: */
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		glBegin(GL_QUADS);
		glColor<4>(surfaceColor.getComponents());
		glVertex(points[0]);
		glVertex(points[1]);
		glVertex(points[3]);
		glVertex(points[2]);
		glEnd();
		
		/* Draw the plane's grid: */
		glBlendFunc(GL_ONE,GL_ONE);
		glLineWidth(1.0f);
		glBegin(GL_LINES);
		glColor<4>(gridColor.getComponents());
		for(int x=0;x<=numLines[0];++x)
			{
			glVertex(Geometry::affineCombination(points[0],points[1],Scalar(x)/Scalar(numLines[0])));
			glVertex(Geometry::affineCombination(points[2],points[3],Scalar(x)/Scalar(numLines[0])));
			}
		for(int y=0;y<=numLines[1];++y)
			{
			glVertex(Geometry::affineCombination(points[0],points[2],Scalar(y)/Scalar(numLines[1])));
			glVertex(Geometry::affineCombination(points[1],points[3],Scalar(y)/Scalar(numLines[1])));
			}
		glEnd();
		
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

void PlanePrimitive::initContext(GLContextData& contextData) const
	{
	/* Create a data item and store it in the context: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	}
