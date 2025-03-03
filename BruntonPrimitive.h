/***********************************************************************
BruntonPrimitive - Class for planes extracted from point clouds, with
additional direct visualization of strike and dip angles.
Copyright (c) 2009-2020 Oliver Kreylos

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

#ifndef BRUNTONPRIMITIVE_INCLUDED
#define BRUNTONPRIMITIVE_INCLUDED

#include <SceneGraph/GroupNode.h>

#include "PlanePrimitive.h"

class BruntonPrimitive:public PlanePrimitive
	{
	/* Elements: */
	protected:
	#if USE_COLLABORATION
	static DataType::TypeID type; // Data type used by this primitive class
	#endif
	SceneGraph::GroupNodePointer root; // Root node of the brunton visualization
	
	/* Constructors and destructors: */
	public:
	BruntonPrimitive(void) // Dummy constructor
		{
		}
	BruntonPrimitive(const LidarOctree* octree,const Vector& translation); // Creates Brunton by processing selected points from the given octree
	BruntonPrimitive(IO::File& file,const Vector& translation) // Creates a Brunton primitive by reading from a binary file
		:PlanePrimitive(file,translation)
		{
		/* Build the Brunton visualization: */
		buildBrunton();
		}
	BruntonPrimitive(Cluster::MulticastPipe* pipe) // Creates a Brunton primitive by reading from an intra-cluster pipe
		:PlanePrimitive(pipe)
		{
		/* Build the Brunton visualization: */
		buildBrunton();
		}
	virtual ~BruntonPrimitive(void);
	
	/* Methods from class Primitive: */
	#if USE_COLLABORATION
	static void registerType(DataType& dataType);
	static DataType::TypeID getClassType(void)
		{
		return type;
		}
	virtual DataType::TypeID getType(void) const;
	#endif
	
	/* New methods: */
	void buildBrunton(void); // Creates the Brunton visualization
	};

#endif
