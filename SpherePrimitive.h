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

#ifndef SPHEREPRIMITIVE_INCLUDED
#define SPHEREPRIMITIVE_INCLUDED

#include <GL/gl.h>
#include <GL/GLObject.h>

#include "PointPrimitive.h"

/* Forward declarations: */
class LidarOctree;

class SpherePrimitive:public PointPrimitive,public GLObject
	{
	/* Embedded classes: */
	protected:
	struct DataItem:public GLObject::DataItem
		{
		/* Elements: */
		public:
		GLuint displayListId; // ID of the display list containing the sphere's visual representation
		unsigned int version; // Version number of the sphere's visual representation
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	/* Elements: */
	#if USE_COLLABORATION
	static DataType::TypeID type; // Data type used by this primitive class
	#endif
	Scalar radius; // Sphere radius
	
	/* Constructors and destructors: */
	public:
	SpherePrimitive(void) // Dummy constructor
		{
		}
	SpherePrimitive(const LidarOctree* octree,const Vector& translation); // Creates sphere by processing selected points from the given octree
	SpherePrimitive(IO::File& file,const Vector& translation) // Creates a sphere primitive by reading from a binary file
		{
		/* Read the primitive: */
		SpherePrimitive::read(file,translation);
		}
	SpherePrimitive(Cluster::MulticastPipe* pipe) // Creates a sphere primitive by reading from an intra-cluster pipe
		{
		/* Read the primitive: */
		SpherePrimitive::read(pipe);
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
	
	virtual DragState* pick(const Point& pickPoint,Scalar& maxPickDistance2);
	virtual void glRenderAction(GLContextData& contextData) const;
	virtual void glRenderActionTransparent(GLContextData& contextData) const;
	
	/* Methods from class GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	};

#endif
