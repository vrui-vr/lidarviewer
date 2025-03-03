/***********************************************************************
PointSelectorTool - Tool class to select/deselect points in a LiDAR
point cloud using a selection sphere.
Copyright (c) 2005-2023 Oliver Kreylos

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

#ifndef POINTSELECTORTOOL_INCLUDED
#define POINTSELECTORTOOL_INCLUDED

#include <Vrui/Types.h>
#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/TransparentObject.h>

#include "LidarViewer.h"

class LidarViewer::PointSelectorTool:public Vrui::Tool,public Vrui::Application::Tool<LidarViewer>,public Vrui::TransparentObject
	{
	friend class Vrui::GenericToolFactory<PointSelectorTool>;
	
	/* Embedded classes: */
	private:
	typedef Vrui::GenericToolFactory<PointSelectorTool> Factory; // Type for factories for this tool class
	
	/* Elements: */
	static Factory* factory; // Pointer to the factory object for this class
	Vrui::Scalar radius; // Radius of selection sphere in physical coordinates
	LidarViewer::SelectorMode selectorMode; // Current selection mode
	unsigned int initMask; // Bit mask of which tool settings have been initialized
	bool active; // Flag whether the selector is active (selecting/deselecting points)
	
	/* Constructors and destructors: */
	public:
	static void initClass(Vrui::ToolFactory* parentFactory); // Initializes the point selector tool class
	PointSelectorTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
	
	/* Methods from class Vrui::Tool: */
	virtual void configure(const Misc::ConfigurationFileSection& configFileSection);
	virtual void storeState(Misc::ConfigurationFileSection& configFileSection) const;
	virtual void initialize(void);
	virtual void deinitialize(void);
	virtual const Vrui::ToolFactory* getFactory(void) const;
	virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
	virtual void frame(void);
	
	/* Methods from class Vrui::TransparentObject: */
	virtual void glRenderActionTransparent(GLContextData& contextData) const;
	
	/* New methods: */
	void update(void); // Updates the tool settings with new application defaults
	};

#endif
