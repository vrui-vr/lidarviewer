/***********************************************************************
ProjectorTool - Transform tool class to position a virtual input device
at the intersection of a ray and a LiDAR octree.
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

#ifndef PROJECTORTOOL_INCLUDED
#define PROJECTORTOOL_INCLUDED

#include <Vrui/TransformTool.h>
#include <Vrui/GenericToolFactory.h>

#include "LidarViewer.h"

class LidarViewer::ProjectorTool:public Vrui::TransformTool,public Vrui::Application::Tool<LidarViewer>
	{
	friend class Vrui::GenericToolFactory<ProjectorTool>;
	
	/* Embedded classes: */
	private:
	typedef Vrui::GenericToolFactory<ProjectorTool> Factory; // Type for factories for this tool class
	
	/* Elements: */
	static Factory* factory; // Pointer to the factory object for this class
	
	/* Constructors and destructors: */
	public:
	static void initClass(void); // Initializes the projector tool class
	ProjectorTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
	
	/* Methods from class Vrui::Tool: */
	virtual void initialize(void);
	virtual const Vrui::ToolFactory* getFactory(void) const;
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	};

#endif
