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

#ifndef POINTPRIMITIVE_INCLUDED
#define POINTPRIMITIVE_INCLUDED

#include <Geometry/Point.h>

#include "Primitive.h"

/* Forward declarations: */
class LinePrimitive;
class PlanePrimitive;

class PointPrimitive:public Primitive
	{
	/* Elements: */
	protected:
	#if USE_COLLABORATION
	static DataType::TypeID type; // Data type used by this primitive class
	#endif
	Point point; // The extracted point
	
	/* Constructors and destructors: */
	public:
	PointPrimitive(void) // Dummy constructor
		{
		};
	PointPrimitive(const PlanePrimitive* const ps[3],const Vector& translation); // Creates point primitive by intersecting the three given plane primitives
	PointPrimitive(const PlanePrimitive* p,const LinePrimitive* l,const Vector& translation); // Creates point primitive by intersecting the given plane and line primitives
	PointPrimitive(IO::File& file,const Vector& translation) // Creates a point primitive by reading from a binary file
		{
		/* Read the primitive: */
		PointPrimitive::read(file,translation);
		}
	PointPrimitive(Cluster::MulticastPipe* pipe) // Creates a point primitive by reading from an intra-cluster pipe
		{
		/* Read the primitive: */
		PointPrimitive::read(pipe);
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
	virtual DragState* pick(const Point& pickPoint,Scalar& maxPickDistance2);
	virtual void glRenderAction(GLContextData& contextData) const;
	};

#endif
