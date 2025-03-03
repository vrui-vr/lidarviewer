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

#ifndef CYLINDERPRIMITIVE_INCLUDED
#define CYLINDERPRIMITIVE_INCLUDED

#include <GL/gl.h>
#include <GL/GLObject.h>

#include "LinePrimitive.h"

/* Forward declarations: */
class LidarOctree;

class CylinderPrimitive:public LinePrimitive,public GLObject
	{
	/* Embedded classes: */
	protected:
	struct DataItem:public GLObject::DataItem
		{
		/* Elements: */
		public:
		GLuint displayListId; // ID of the display list containing the cylinder's visual representation
		unsigned int version; // Version number of the cylinder's visual representation
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	/* Elements: */
	#if USE_COLLABORATION
	static DataType::TypeID type; // Data type used by this primitive class
	#endif
	Scalar radius; // Cylinder radius
	int numLines[2]; // Number of grid lines to render along the cylinder's x and y directions to achieve a mostly square grid
	
	/* Constructors and destructors: */
	public:
	CylinderPrimitive(void) // Dummy constructor
		{
		};
	CylinderPrimitive(const LidarOctree* octree,const Vector& translation); // Creates cylinder by processing selected points from the given octree
	CylinderPrimitive(IO::File& file,const Vector& translation) // Creates a cylinder primitive by reading from a binary file
		{
		/* Read the primitive: */
		CylinderPrimitive::read(file,translation);
		}
	CylinderPrimitive(Cluster::MulticastPipe* pipe) // Creates a cylinder primitive by reading from an intra-cluster pipe
		{
		/* Read the primitive: */
		CylinderPrimitive::read(pipe);
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
	virtual void glRenderActionTransparent(GLContextData& contextData) const;
	
	/* Methods from class GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	
	/* New methods: */
	Scalar getRadius(void) const // Returns the cylinder's radius
		{
		return radius;
		}
	const int* getNumLines(void) const // Returns the number of grid lines
		{
		return numLines;
		}
	};

#endif
