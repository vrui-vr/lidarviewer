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

#include "Primitive.h"

#include <Misc/StringMarshaller.h>
#include <IO/File.h>
#include <Cluster/MulticastPipe.h>
#if USE_COLLABORATION
#include <Collaboration2/DataType.icpp>
#endif

/*************************************
Methods of class Primitive::DragState:
*************************************/

Primitive::DragState::~DragState(void)
	{
	}

/**********************************
Static elements of class Primitive:
**********************************/

#if USE_COLLABORATION

DataType::TypeID Primitive::scalarType=DataType::TypeID(-1);
DataType::TypeID Primitive::pointType=DataType::TypeID(-1);
DataType::TypeID Primitive::vectorType=DataType::TypeID(-1);
DataType::TypeID Primitive::type=DataType::TypeID(-1);

#endif

/**************************
Methods of class Primitive:
**************************/

Primitive::Primitive(void)
	:
	 #if USE_COLLABORATION
	 objectId(0),
	 #endif
	 numPoints(0),rms(0),
	 surfaceColor(0.6f,0.6f,0.1f,0.5f),
	 gridColor(0.2f,0.2f,0.2f),
	 version(1)
	{
	}

Primitive::Primitive(IO::File& file,const Vector& translation)
	:
	 #if USE_COLLABORATION
	 objectId(0),
	 #endif
	 surfaceColor(0.6f,0.6f,0.1f,0.5f),
	 gridColor(0.2f,0.2f,0.2f),
	 version(1)
	{
	/* Read the primitive: */
	Primitive::read(file,translation);
	}

Primitive::Primitive(Cluster::MulticastPipe* pipe)
	:
	 #if USE_COLLABORATION
	 objectId(0),
	 #endif
	 surfaceColor(0.6f,0.6f,0.1f,0.5f),
	 gridColor(0.2f,0.2f,0.2f),
	 version(1)
	{
	/* Read the primitive: */
	Primitive::read(pipe);
	}

Primitive::~Primitive(void)
	{
	}

void Primitive::setLabel(const std::string& newLabel)
	{
	label=newLabel;
	}

void Primitive::write(IO::File& file,const Vector& translation) const
	{
	/* Write the number of points and the approximation RMS: */
	file.write(numPoints);
	file.write(rms);
	
	/* Write the primitive label: */
	Misc::writeCppString(label,file);
	}

void Primitive::read(IO::File& file,const Vector& translation)
	{
	/* Read the number of points and the approximation RMS: */
	file.read(numPoints);
	file.read(rms);
	
	/* Read the primitive label: */
	Misc::readCppString(file,label);
	}

void Primitive::write(Cluster::MulticastPipe* pipe) const
	{
	/* Write the number of points and the approximation RMS: */
	pipe->write(numPoints);
	pipe->write(rms);
	
	/* Write the primitive label: */
	Misc::writeCppString(label,*pipe);
	}

void Primitive::read(Cluster::MulticastPipe* pipe)
	{
	/* Read the number of points and the approximation RMS: */
	pipe->read(numPoints);
	pipe->read(rms);
	
	/* Read the primitive label: */
	Misc::readCppString(*pipe,label);
	}

#if USE_COLLABORATION

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"

void Primitive::registerType(DataType& dataType)
	{
	/* Register base types: */
	scalarType=dataType.getAtomicType<Misc::Float64>();
	pointType=dataType.createFixedArray(3,scalarType);
	vectorType=dataType.createFixedArray(3,scalarType);
	
	/* Register the primitive structure: */
	DataType::StructureElement elements[]=
		{
		{DataType::getAtomicType<Misc::UInt64>(),offsetof(Primitive,numPoints)},
		{scalarType,offsetof(Primitive,rms)},
		{DataType::String,offsetof(Primitive,label)}
		};
	type=dataType.createStructure(3,elements,sizeof(Primitive));
	}

#pragma GCC diagnostic pop

DataType::TypeID Primitive::getType(void) const
	{
	return type;
	}

void Primitive::setObjectId(KoinoniaProtocol::ObjectID newObjectId)
	{
	objectId=newObjectId;
	}

#endif

void Primitive::setSurfaceColor(const Primitive::Color& newSurfaceColor)
	{
	surfaceColor=newSurfaceColor;
	++version;
	}

void Primitive::setGridColor(const Primitive::Color& newGridColor)
	{
	gridColor=newGridColor;
	++version;
	}
Primitive::DragState* Primitive::pick(const Primitive::Point& pickPoint,Primitive::Scalar& maxPickDistance2)
	{
	/* Base primitives cannot be picked: */
	return 0;
	}

void Primitive::drag(Primitive::DragState* dragState,const Primitive::Point& dragPoint)
	{
	/* Don't drag by default, even if the pick succeeded */
	}

void Primitive::glRenderAction(GLContextData& contextData) const
	{
	/* Don't draw anything */
	}

void Primitive::glRenderActionTransparent(GLContextData& contextData) const
	{
	/* Don't draw anything */
	}
