/***********************************************************************
Primitive - Base class for geometric primitives (planes, spheres, ...)
that can be extracted from point clouds.
Copyright (c) 2007-2022 Oliver Kreylos

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

#ifndef PRIMITIVE_INCLUDED
#define PRIMITIVE_INCLUDED

#include "Config.h"

#include <string>
#include <Misc/SizedTypes.h>
#include <Misc/RGBA.h>
#if USE_COLLABORATION
#include <Collaboration2/DataType.h>
#include <Collaboration2/Plugins/KoinoniaProtocol.h>
#endif

/* Forward declarations: */
namespace IO {
class File;
}
namespace Cluster {
class MulticastPipe;
}
namespace Geometry {
template <class ScalarParam,int dimensionParam>
class Point;
template <class ScalarParam,int dimensionParam>
class Vector;
}
class GLContextData;

/* Namespace shortcuts: */
#if USE_COLLABORATION
using Collab::DataType;
using Collab::Plugins::KoinoniaProtocol;
#endif

class Primitive
	{
	/* Embedded classes: */
	public:
	typedef Misc::Float64 Scalar; // Scalar type for primitive parameters
	typedef Geometry::Point<Scalar,3> Point; // Point type for primitives
	typedef Geometry::Vector<Scalar,3> Vector; // Vector type for primitives
	typedef Misc::RGBA<Misc::Float32> Color; // Type for colors with opacity values
	
	class DragState // Base class to maintain state of an ongoing dragging operation
		{
		/* Elements: */
		protected:
		Primitive* primitive; // Pointer to the primitive being dragged
		
		/* Constructors and destructors: */
		public:
		DragState(Primitive* sPrimitive)
			:primitive(sPrimitive)
			{
			}
		virtual ~DragState(void);
		
		/* Methods: */
		Primitive* getPrimitive(void) // Returns the dragged primitive
			{
			return primitive;
			}
		};
	
	/* Elements: */
	protected:
	#if USE_COLLABORATION
	static DataType::TypeID scalarType,pointType,vectorType; // Base types used by primitive classes
	static DataType::TypeID type; // Data type used by this primitive class
	KoinoniaProtocol::ObjectID objectId; // Shared ID of this primitive with the Koinonia protocol client
	#endif
	Misc::UInt64 numPoints; // Number of points used to construct the primitive
	Scalar rms; // Root-mean square residual of the primitive with respect to its source points
	private:
	std::string label; // A label for the extracted primitive
	protected:
	Color surfaceColor; // Color to render the primitive's surface
	Color gridColor; // Color to render the primitive's grid
	unsigned int version; // Version number to synchronize application and graphics state
	
	/* Constructors and destructors: */
	Primitive(void); // Dummy constructor
	Primitive(IO::File& file,const Vector& translation); // Creates a primitive by reading from a binary file
	Primitive(Cluster::MulticastPipe* pipe); // Creates a primitive by reading from an intra-cluster pipe
	public:
	virtual ~Primitive(void); // Destroys the primitive
	
	/* Methods: */
	Misc::UInt64 getNumPoints(void) const // Returns the number of points used to construct the primitive
		{
		return numPoints;
		}
	Scalar getRms(void) const // Returns the primitive's extraction residual
		{
		return rms;
		}
	const std::string& getLabel(void) const // Returns the primitive's label
		{
		return label;
		}
	virtual void setLabel(const std::string& newLabel); // Sets the primitive's label
	
	/* File I/O methods: */
	virtual void write(IO::File& file,const Vector& translation) const; // Writes a primitive to a binary file
	virtual void read(IO::File& file,const Vector& translation); // Reads a primitive from a binary file
	
	/* Intra-cluster communication methods: */
	virtual void write(Cluster::MulticastPipe* pipe) const; // Writes a primitive to a intra-cluster pipe
	virtual void read(Cluster::MulticastPipe* pipe); // Reads a primitive from an intra-cluster pipe
	
	#if USE_COLLABORATION
	
	/* Collaboration methods: */
	static void registerType(DataType& dataType); // Registers a data type to transmit a primitive with the given data type library
	static DataType::TypeID getClassType(void) // Returns the primitive class's data type
		{
		return type;
		}
	virtual DataType::TypeID getType(void) const; // Returns the primitive's data type
	KoinoniaProtocol::ObjectID getObjectId(void) const // Returns this primitive's Koinonia object ID
		{
		return objectId;
		}
	void setObjectId(KoinoniaProtocol::ObjectID newObjectId); // Sets this primitive's Koinonia object ID
	
	#endif
	
	/* Rendering and interaction methods: */
	virtual void setSurfaceColor(const Color& newSurfaceColor); // Sets the primitive's surface color
	virtual void setGridColor(const Color& newGridColor); // Sets the primitive's grid color
	void invalidate(void) // Explicitly invalidates the primitive's representation
		{
		++version;
		}
	virtual DragState* pick(const Primitive::Point& pickPoint,Primitive::Scalar& maxPickDistance2); // Returns the the result of picking the primitive from the given position, with the given squared maximum distance; updates maxPickDistance2 if valid pick; returns null if no valid pick
	virtual void drag(DragState* dragState,const Primitive::Point& dragPoint); // Drags the primitive using the given dragging state and current dragging tool position
	virtual void glRenderAction(GLContextData& contextData) const; // Draws the primitive during the regular rendering pass
	virtual void glRenderActionTransparent(GLContextData& contextData) const; // Draws the primitive during the transparent rendering pass
	};

#endif
