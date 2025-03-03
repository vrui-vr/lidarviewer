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

#ifndef PRIMITIVEDRAGGERTOOL_INCLUDED
#define PRIMITIVEDRAGGERTOOL_INCLUDED

#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>

#include "Primitive.h"
#include "LidarViewer.h"

class LidarViewer::PrimitiveDraggerTool:public Vrui::Tool,public Vrui::Application::Tool<LidarViewer>
	{
	friend class Vrui::GenericToolFactory<PrimitiveDraggerTool>;
	
	/* Embedded classes: */
	private:
	typedef Vrui::GenericToolFactory<PrimitiveDraggerTool> Factory; // Type for factories for this tool class
	
	/* Elements: */
	static Factory* factory; // Pointer to the factory object for this class
	Primitive::DragState* dragState; // The state of the current drag operation
	double dragStartTime; // Application time at which the current drag operation started
	
	/* Constructors and destructors: */
	public:
	static void initClass(Vrui::ToolFactory* parentFactory); // Initializes the point selector tool class
	PrimitiveDraggerTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
	virtual ~PrimitiveDraggerTool(void);
	
	/* Methods from class Vrui::Tool: */
	virtual const Vrui::ToolFactory* getFactory(void) const;
	virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
	virtual void frame(void);
	};

#endif
