/***********************************************************************
PrimitiveDraggerTool - Tool class to select/deselect and drag extracted
primitives.
Copyright (c) 2005-2020 Oliver Kreylos

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

#include "PrimitiveDraggerTool.h"

#include <Geometry/OrthogonalTransformation.h>

/**********************************************************
Static elements of class LidarViewer::PrimitiveDraggerTool:
**********************************************************/

LidarViewer::PrimitiveDraggerTool::Factory* LidarViewer::PrimitiveDraggerTool::factory=0;

/**************************************************
Methods of class LidarViewer::PrimitiveDraggerTool:
**************************************************/

void LidarViewer::PrimitiveDraggerTool::initClass(Vrui::ToolFactory* parentFactory)
	{
	/* Create the projector tool class's factory: */
	Factory* factory=new Factory("LidarPrimitiveDraggerTool","Drag Primitives",parentFactory,*Vrui::getToolManager());
	
	/* Set the point selector tool class's input layout: */
	factory->setNumButtons(1,false);
	
	/* Add the factory to the tool manager: */
	Vrui::getToolManager()->addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	}

LidarViewer::PrimitiveDraggerTool::PrimitiveDraggerTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment),
	 dragState(0)
	{
	}

LidarViewer::PrimitiveDraggerTool::~PrimitiveDraggerTool(void)
	{
	/* Delete a potential leftover dragging state: */
	delete dragState;
	}

const Vrui::ToolFactory* LidarViewer::PrimitiveDraggerTool::getFactory(void) const
	{
	return factory;
	}

void LidarViewer::PrimitiveDraggerTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	/* Check if the button was pressed or released: */
	if(cbData->newButtonState)
		{
		/* Pick all extracted primitives: */
		dragState=application->pickPrimitive(Vrui::getInverseNavigationTransformation().transform(getButtonDevicePosition(0)));
		if(dragState!=0)
			dragStartTime=Vrui::getApplicationTime();
		}
	else
		{
		/* Check if there is an active drag operation: */
		if(dragState!=0)
			{
			/* Check if this was a click rather than a drag: */
			if(Vrui::getApplicationTime()-dragStartTime<0.25)
				{
				/* Toggle the picked primitive's selection state: */
				application->togglePrimitive(dragState->getPrimitive());
				}
			
			/* Delete the pick result to end the drag operation: */
			delete dragState;
			dragState=0;
			}
		}
	}

void LidarViewer::PrimitiveDraggerTool::frame(void)
	{
	/* Bail out if there is no active drag operation: */
	if(dragState==0)
		return;
	
	/* Drag the picked primitive: */
	application->dragPrimitive(dragState,Vrui::getInverseNavigationTransformation().transform(getButtonDevicePosition(0)));
	}
