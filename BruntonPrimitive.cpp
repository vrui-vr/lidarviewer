/***********************************************************************
BruntonPrimitive - Class for planes extracted from point clouds, with
additional direct visualization of strike and dip angles.
Copyright (c) 2009-2021 Oliver Kreylos

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

#include "BruntonPrimitive.h"

#include <stdio.h>
#include <iostream>
#include <Math/Math.h>
#include <Geometry/Point.h>
#include <SceneGraph/TransformNode.h>
#include <SceneGraph/BillboardNode.h>
#include <SceneGraph/ColorNode.h>
#include <SceneGraph/CoordinateNode.h>
#include <SceneGraph/IndexedLineSetNode.h>
#include <SceneGraph/FontStyleNode.h>
#include <SceneGraph/TextNode.h>
#include <SceneGraph/ShapeNode.h>

#include "SceneGraph.h"

/*****************************************
Static elements of class BruntonPrimitive:
*****************************************/

#if USE_COLLABORATION

DataType::TypeID BruntonPrimitive::type=DataType::TypeID(-1);

#endif

/*********************************
Methods of class BruntonPrimitive:
*********************************/

BruntonPrimitive::BruntonPrimitive(const LidarOctree* octree,const Primitive::Vector& translation)
	:PlanePrimitive(octree,translation)
	{
	/* Print the strike and dip angles: */
	Vector n=normal;
	if(n[2]<Scalar(0))
		n=-n;
	Scalar dipAngle=Math::acos(n[2]/Geometry::mag(n));
	Scalar strikeAngle=Math::atan2(n[0],n[1]);
	if(strikeAngle<Scalar(0))
		strikeAngle+=Scalar(2)*Math::Constants<Scalar>::pi;
	std::cout<<"Strike angle: "<<Math::deg(strikeAngle)<<std::endl;
	std::cout<<"Dip angle: "<<Math::deg(dipAngle)<<std::endl;
	
	/* Build the brunton visualization: */
	buildBrunton();
	}

BruntonPrimitive::~BruntonPrimitive(void)
	{
	/* Remove the brunton root node from the scene graph: */
	getSceneGraphRoot().removeChild(*root);
	}

#if USE_COLLABORATION

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"

void BruntonPrimitive::registerType(DataType& dataType)
	{
	/* Retrieve the base primitive's structure elements: */
	std::vector<DataType::StructureElement> elements=dataType.getStructureElements(PlanePrimitive::type);
	
	/* Extend the base type: */
	// Nothing to extend, actually...
	
	/* Register the extended type: */
	type=dataType.createStructure(elements,sizeof(BruntonPrimitive));
	}

#pragma GCC diagnostic pop

DataType::TypeID BruntonPrimitive::getType(void) const
	{
	return type;
	}

#endif

void BruntonPrimitive::buildBrunton(void)
	{
	/* Create the root node: */
	SceneGraph::TransformNode* rootT=new SceneGraph::TransformNode;
	root=rootT;
	getSceneGraphRoot().addChild(*root);
	
	/* Calculate the plane primitive's scale: */
	Scalar bruntonScale=Math::sqrt(Geometry::dist(getCorner(2),getCorner(0))*Geometry::dist(getCorner(3),getCorner(1)));
	
	/* Calculate the plane primitive's dip angle and strike vector: */
	Vector n=normal;
	if(n[2]<Scalar(0))
		n=-n;
	n.normalize();
	Vector strike=n;
	Scalar dipAngle=Math::acos(strike[2]/Geometry::mag(strike));
	strike[2]=Scalar(0);
	strike.normalize();
	Scalar strikeAngle=Math::atan2(-strike[0],strike[1]);
	
	/* Set the root node's transformation: */
	rootT->translation.setValue(center-Point::origin);
	
	/* Create the dip and strike indicator: */
	SceneGraph::ShapeNode* s1=new SceneGraph::ShapeNode;
	rootT->addChildren.appendValue(s1);
	{
	SceneGraph::IndexedLineSetNode* ils=new SceneGraph::IndexedLineSetNode;
	s1->geometry.setValue(ils);
	
	SceneGraph::ColorNode* color=new SceneGraph::ColorNode;
	ils->color.setValue(color);
	color->color.appendValue(SceneGraph::Color(0.0f,0.5f,1.0f));
	color->color.appendValue(SceneGraph::Color(0.0f,1.0f,0.5f));
	color->update();
	
	SceneGraph::CoordinateNode* coord=new SceneGraph::CoordinateNode;
	ils->coord.setValue(coord);
	coord->point.appendValue(Point::origin);
	coord->point.appendValue(Point::origin+n*bruntonScale);
	coord->point.appendValue(Point::origin+strike*bruntonScale);
	coord->update();
	
	ils->coordIndex.appendValue(0);
	ils->coordIndex.appendValue(1);
	ils->coordIndex.appendValue(-1);
	ils->coordIndex.appendValue(0);
	ils->coordIndex.appendValue(2);
	
	ils->colorPerVertex.setValue(false);
	ils->lineWidth.setValue(3.0f);
	ils->update();
	}
	s1->update();
	
	SceneGraph::ShapeNode* s2=new SceneGraph::ShapeNode;
	rootT->addChildren.appendValue(s2);
	{
	SceneGraph::IndexedLineSetNode* ils=new SceneGraph::IndexedLineSetNode;
	s2->geometry.setValue(ils);
	
	SceneGraph::ColorNode* color=new SceneGraph::ColorNode;
	ils->color.setValue(color);
	color->color.appendValue(SceneGraph::Color(0.0f,0.5f,1.0f));
	color->color.appendValue(SceneGraph::Color(0.0f,0.5f,1.0f));
	color->color.appendValue(SceneGraph::Color(0.0f,1.0f,0.5f));
	color->color.appendValue(SceneGraph::Color(0.0f,1.0f,0.5f));
	color->update();
	
	SceneGraph::CoordinateNode* coord=new SceneGraph::CoordinateNode;
	ils->coord.setValue(coord);
	
	coord->point.appendValue(Point::origin);
	coord->point.appendValue(Point(0,0,bruntonScale));
	coord->point.appendValue(Point(0,bruntonScale,0));
	ils->coordIndex.appendValue(0);
	ils->coordIndex.appendValue(1);
	ils->coordIndex.appendValue(-1);
	for(Scalar a=Scalar(0);a<dipAngle;a+=Math::rad(Scalar(10)))
		{
		ils->coordIndex.appendValue(coord->point.getNumValues());
		coord->point.appendValue(Point::origin+(Vector(0,0,1)*Math::cos(a)+strike*Math::sin(a))*(bruntonScale*Scalar(0.9)));
		}
	ils->coordIndex.appendValue(coord->point.getNumValues());
	coord->point.appendValue(Point::origin+(Vector(0,0,1)*Math::cos(dipAngle)+strike*Math::sin(dipAngle))*(bruntonScale*Scalar(0.9)));
	ils->coordIndex.appendValue(-1);
	
	ils->coordIndex.appendValue(0);
	ils->coordIndex.appendValue(2);
	ils->coordIndex.appendValue(-1);
	Scalar aInc=Math::rad(Scalar(10));
	if(strikeAngle<Scalar(0))
		aInc=-aInc;
	for(Scalar a=Scalar(0);Math::abs(a)<Math::abs(strikeAngle);a+=aInc)
		{
		ils->coordIndex.appendValue(coord->point.getNumValues());
		coord->point.appendValue(Point::origin+Vector(-Math::sin(a),Math::cos(a),0)*(bruntonScale*Scalar(0.9)));
		}
	ils->coordIndex.appendValue(coord->point.getNumValues());
	coord->point.appendValue(Point::origin+Vector(-Math::sin(strikeAngle),Math::cos(strikeAngle),0)*(bruntonScale*Scalar(0.9)));
	
	coord->update();
	
	ils->colorPerVertex.setValue(false);
	ils->lineWidth.setValue(1.0f);
	ils->update();
	}
	s2->update();
	
	SceneGraph::TransformNode* t2=new SceneGraph::TransformNode;
	rootT->addChildren.appendValue(t2);
	t2->translation.setValue((Vector(0,0,1)*Math::cos(Math::div2(dipAngle))+strike*Math::sin(Math::div2(dipAngle)))*bruntonScale);
	{
	SceneGraph::BillboardNode* bb=new SceneGraph::BillboardNode;
	t2->addChildren.appendValue(bb);
	bb->axisOfRotation.setValue(Vector::zero);
	{
	SceneGraph::ShapeNode* s=new SceneGraph::ShapeNode;
	bb->addChildren.appendValue(s);
	
	SceneGraph::TextNode* text=new SceneGraph::TextNode;
	s->geometry.setValue(text);
	
	SceneGraph::FontStyleNode* fs=new SceneGraph::FontStyleNode;
	text->fontStyle.setValue(fs);
	fs->size.setValue(bruntonScale*Scalar(0.25));
	fs->justify.appendValue("MIDDLE");
	fs->justify.appendValue("MIDDLE");
	
	fs->update();
	
	char buffer[40];
	snprintf(buffer,sizeof(buffer),"%.2f",Math::deg(dipAngle));
	text->string.appendValue(buffer);
	text->update();
	
	s->update();
	}
	bb->update();
	}
	t2->update();
	
	SceneGraph::TransformNode* t3=new SceneGraph::TransformNode;
	rootT->addChildren.appendValue(t3);
	t3->translation.setValue(Vector(-Math::sin(Math::div2(strikeAngle)),Math::cos(Math::div2(strikeAngle)),0)*bruntonScale);
	{
	SceneGraph::BillboardNode* bb=new SceneGraph::BillboardNode;
	t3->addChildren.appendValue(bb);
	bb->axisOfRotation.setValue(Vector::zero);
	{
	SceneGraph::ShapeNode* s=new SceneGraph::ShapeNode;
	bb->addChildren.appendValue(s);
	
	SceneGraph::TextNode* text=new SceneGraph::TextNode;
	s->geometry.setValue(text);
	
	SceneGraph::FontStyleNode* fs=new SceneGraph::FontStyleNode;
	text->fontStyle.setValue(fs);
	fs->size.setValue(bruntonScale*Scalar(0.25));
	fs->justify.appendValue("MIDDLE");
	fs->justify.appendValue("MIDDLE");
	
	fs->update();
	
	char buffer[40];
	Scalar sa=-Math::deg(strikeAngle);
	if(sa<Scalar(0))
		sa+=360.0;
	snprintf(buffer,sizeof(buffer),"%.2f",sa);
	text->string.appendValue(buffer);
	text->update();
	
	s->update();
	}
	bb->update();
	}
	t3->update();
	
	rootT->update();
	}
