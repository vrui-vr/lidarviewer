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

#ifndef LINEPRIMITIVE_INCLUDED
#define LINEPRIMITIVE_INCLUDED

#include <Geometry/ComponentArray.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>

#include "Primitive.h"

/* Forward declarations: */
class LidarOctree;
class PlanePrimitive;

class LinePrimitive:public Primitive
	{
	/* Embedded classes: */
	public:
	typedef Geometry::ComponentArray<Scalar,2> Interval; // Type for 1D closed intervals
	
	protected:
	struct DragState:public Primitive::DragState
		{
		/* Embedded classes: */
		public:
		enum PickedPart // Enumerated type for picked line parts
			{
			Lower,Upper,Line
			};
		
		/* Elements: */
		PickedPart pickedPart; // The picked line part
		Scalar pickOffset; // Offset from initial pick position to picked line part
		
		/* Constructors and destructors: */
		DragState(LinePrimitive* sPrimitive,PickedPart sPickedPart,Scalar sPickOffset)
			:Primitive::DragState(sPrimitive),
			 pickedPart(sPickedPart),pickOffset(sPickOffset)
			{
			}
		};
	
	/* Elements: */
	#if USE_COLLABORATION
	static DataType::TypeID type; // Data type used by this primitive class
	#endif
	Point center; // Line's center point as extracted
	Vector axis; // Normalized line direction
	Scalar length; // Original line length as extracted
	Interval extents; // Extents of line's visual representation relative to the center
	
	/* Constructors and destructors: */
	public:
	LinePrimitive(void) // Dummy constructor
		{
		};
	LinePrimitive(const LidarOctree* octree,const Vector& translation); // Creates line by processing selected points from the given octree
	LinePrimitive(const PlanePrimitive* const ps[2],const Vector& translation); // Creates line primitive by intersecting the two given plane primitives
	LinePrimitive(IO::File& file,const Vector& translation) // Creates a line primitive by reading from a binary file
		{
		/* Read the primitive: */
		LinePrimitive::read(file,translation);
		}
	LinePrimitive(Cluster::MulticastPipe* pipe) // Creates a line primitive by reading from an intra-cluster pipe
		{
		/* Read the primitive: */
		LinePrimitive::read(pipe);
		}
	
	/* Methods from class Primitive: */
	virtual void write(IO::File& file,const Vector& translation) const;
	virtual void read(IO::File& file,const Vector& translation);
	virtual void write(Cluster::MulticastPipe* pipe) const;
	virtual void read(Cluster::MulticastPipe* pipe);
	
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
	virtual void glRenderAction(GLContextData& contextData) const;
	
	/* New methods: */
	const Point& getCenter(void) const // Returns the line's center point
		{
		return center;
		};
	const Vector& getAxis(void) const // Returns the line's axis direction
		{
		return axis;
		};
	Scalar getLength(void) const // Returns the line's length
		{
		return length;
		}
	const Interval& getExtents(void) const // Returns the extents of the line's visual representation
		{
		return extents;
		}
	};

#endif
