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

#ifndef PLANEPRIMITIVE_INCLUDED
#define PLANEPRIMITIVE_INCLUDED

#include <Geometry/ComponentArray.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/Plane.h>
#include <GL/gl.h>
#include <GL/GLObject.h>

#include "Primitive.h"

/* Forward declarations: */
class LidarOctree;

class PlanePrimitive:public Primitive,public GLObject
	{
	/* Embedded classes: */
	public:
	typedef Geometry::ComponentArray<Scalar,2> Interval; // Type for 1D closed intervals
	typedef Geometry::Plane<Scalar,3> Plane; // Type for plane equations
	
	protected:
	struct DragState:public Primitive::DragState
		{
		/* Embedded classes: */
		public:
		enum PickedPart // Enumerated type for pickable primitive parts
			{
			Corner,Edge,Face
			};
		
		/* Elements: */
		public:
		PickedPart pickedPart; // The type of the part of the primitive that was picked
		int pickedPartIndex; // The index of the part of the primitive that was picked
		Vector offset; // Offset between initial position of picked part and picking device in physical space
		
		/* Constructors and destructors: */
		DragState(PlanePrimitive* sPrimitive,PickedPart sPickedPart,int sPickedPartIndex,const Vector& sOffset)
			:Primitive::DragState(sPrimitive),
			 pickedPart(sPickedPart),pickedPartIndex(sPickedPartIndex),offset(sOffset)
			{
			}
		};
	
	struct DataItem:public GLObject::DataItem
		{
		/* Elements: */
		public:
		GLuint displayListId; // ID of the display list containing the plane's visual representation
		unsigned int version; // Version number of the plane's visual representation
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	/* Elements: */
	#if USE_COLLABORATION
	static DataType::TypeID type; // Data type used by this primitive class
	#endif
	Point center; // Center point of the extracted plane
	Vector normal; // Normal vector of the extracted plane
	Vector axes[2]; // Axes of the extracted plane's local coordinate system
	Interval extents[2]; // Extents of the plane's visual representation along the x and y axes, respectively
	int numLines[2]; // Number of grid lines to render along the plane's x and y directions to achieve a mostly square grid
	
	/* Constructors and destructors: */
	public:
	PlanePrimitive(void) // Dummy constructor
		{
		};
	PlanePrimitive(const LidarOctree* octree,const Vector& translation); // Creates plane by processing selected points from the given octree
	PlanePrimitive(IO::File& file,const Vector& translation) // Creates a plane primitive by reading from a binary file
		{
		/* Read the primitive: */
		PlanePrimitive::read(file,translation);
		}
	PlanePrimitive(Cluster::MulticastPipe* pipe) // Creates a plane primitive by reading from an intra-cluster pipe
		{
		/* Read the primitive: */
		PlanePrimitive::read(pipe);
		}
	
	/* Methods from class Primitive: */
	virtual void write(IO::File& file,const Vector& translation) const; // Writes a primitive to a binary file
	virtual void read(IO::File& file,const Vector& translation); // Reads a primitive from a binary file
	virtual void write(Cluster::MulticastPipe* pipe) const; // Writes a primitive to a intra-cluster pipe
	virtual void read(Cluster::MulticastPipe* pipe); // Reads a primitive from an intra-cluster pipe
	
	#if USE_COLLABORATION
	static void registerType(DataType& dataType);
	static DataType::TypeID getClassType(void)
		{
		return type;
		}
	virtual DataType::TypeID getType(void) const;
	#endif
	
	virtual Primitive::DragState* pick(const Point& pickPoint,Scalar& maxPickDistance2);
	virtual void drag(Primitive::DragState* dragState,const Point& dragPoint);
	virtual void glRenderActionTransparent(GLContextData& contextData) const;
	
	/* Methods from class GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	
	/* New methods: */
	const Point& getCenter(void) const // Returns the extracted plane's center
		{
		return center;
		}
	const Vector& getNormal(void) const // Returns the extracted plane's normal vector
		{
		return normal;
		}
	Plane getPlane(void) const // Returns the equation of the extracted plane
		{
		return Plane(normal,center);
		};
	const Vector* getAxes(void) const // Returns the axes of the extracted plane's local coordinate system
		{
		return axes;
		}
	const Interval* getExtents(void) const // Returns the extents of the extracted plane's visual representation
		{
		return extents;
		}
	Point getCorner(int index) const // Returns one of the rectangle's corners
		{
		Point result=center;
		for(int i=0;i<2;++i)
			result+=axes[i]*((index&(0x1<<i))?extents[i][1]:extents[i][0]);
		return result;
		};
	const int* getNumLines(void) const // Returns the extracted plane's number of lines
		{
		return numLines;
		}
	};

#endif
