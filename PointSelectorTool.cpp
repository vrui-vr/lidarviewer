/***********************************************************************
PointSelectorTool - Tool class to select/deselect points in a LiDAR
point cloud using a selection sphere.
Copyright (c) 2005-2024 Oliver Kreylos

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

#include "PointSelectorTool.h"

#include <string>
#include <Misc/StdError.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <Geometry/OrthogonalTransformation.h>
#include <GL/gl.h>
#include <GL/GLMatrixTemplates.h>
#include <GL/GLContextData.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>

#include "LidarOctree.h"

/*******************************************************
Static elements of class LidarViewer::PointSelectorTool:
*******************************************************/

LidarViewer::PointSelectorTool::Factory* LidarViewer::PointSelectorTool::factory=0;

/***********************************************
Methods of class LidarViewer::PointSelectorTool:
***********************************************/

void LidarViewer::PointSelectorTool::initClass(Vrui::ToolFactory* parentFactory)
	{
	/* Create the projector tool class's factory: */
	Factory* factory=new Factory("LidarPointSelectorTool","Select Points",parentFactory,*Vrui::getToolManager());
	
	/* Set the point selector tool class's input layout: */
	factory->setNumButtons(1,false);
	
	/* Add the factory to the tool manager: */
	Vrui::getToolManager()->addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	}

LidarViewer::PointSelectorTool::PointSelectorTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment),
	 radius(0),selectorMode(Add),initMask(0x0U),
	 active(false)
	{
	}

void LidarViewer::PointSelectorTool::configure(const Misc::ConfigurationFileSection& configFileSection)
	{
	/* Read tool settings from configuration file section: */
	if(configFileSection.hasTag("./radius"))
		{
		radius=configFileSection.retrieveValue<Vrui::Scalar>("./radius");
		initMask|=0x1U;
		}
	if(configFileSection.hasTag("./selectorMode"))
		{
		std::string selectorModeName=configFileSection.retrieveString("./selectorMode");
		if(selectorModeName=="Add")
			selectorMode=LidarViewer::Add;
		else if(selectorModeName=="Subtract")
			selectorMode=LidarViewer::Subtract;
		else
			throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Invalid selector mode %s",selectorModeName.c_str());
		initMask|=0x2U;
		}
	}

void LidarViewer::PointSelectorTool::storeState(Misc::ConfigurationFileSection& configFileSection) const
	{
	/* Write tool settings to the configuration file section: */
	configFileSection.storeValue<Vrui::Scalar>("./radius",radius);
	configFileSection.storeString("./selectorMode",selectorMode==LidarViewer::Add?"Add":"Subtract");
	}

void LidarViewer::PointSelectorTool::initialize(void)
	{
	/* Copy uninitialized tool settings from LidarViewer's defaults: */
	if((initMask&0x01U)==0x0U)
		radius=application->defaultSelectorRadius;
	if((initMask&0x02U)==0x0U)
		selectorMode=application->defaultSelectorMode;
	
	/* Add this tool to the application's point selector tool list: */
	application->pointSelectorTools.push_back(this);
	}

void LidarViewer::PointSelectorTool::deinitialize(void)
	{
	/* Remove this tool from the application's point selector tool list: */
	for(LidarViewer::PointSelectorToolList::iterator pstIt=application->pointSelectorTools.begin();pstIt!=application->pointSelectorTools.end();++pstIt)
		if(*pstIt==this)
			{
			/* Remove this tool and stop searching: */
			*pstIt=application->pointSelectorTools.back();
			application->pointSelectorTools.pop_back();
			break;
			}
	}

const Vrui::ToolFactory* LidarViewer::PointSelectorTool::getFactory(void) const
	{
	return factory;
	}

void LidarViewer::PointSelectorTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	/* Check if the button was pressed or released: */
	if(cbData->newButtonState)
		{
		/* Start selecting/deselecting points: */
		active=true;
		}
	else
		{
		/* Stop selecting/deselecting points: */
		active=false;
		}
	}

void LidarViewer::PointSelectorTool::frame(void)
	{
	/* Bail out if not active: */
	if(!active)
		return;
	
	/* Create a LiDAR interactor: */
	LidarOctree::Interactor interactor(Vrui::getInverseNavigationTransformation().transform(getButtonDevicePosition(0)),Vrui::getInverseNavigationTransformation().getScaling()*radius);
	
	/* Let all octrees prepare for interaction: */
	for(int i=0;i<application->numOctrees;++i)
		if(application->showOctrees[i])
			application->octrees[i]->interact(interactor);
	
	/* Select or deselect points: */
	switch(selectorMode)
		{
		case LidarViewer::Add:
			for(int i=0;i<application->numOctrees;++i)
				if(application->showOctrees[i])
					application->octrees[i]->selectPoints(interactor);
			break;
		
		case LidarViewer::Subtract:
			for(int i=0;i<application->numOctrees;++i)
				if(application->showOctrees[i])
					application->octrees[i]->deselectPoints(interactor);
			break;
		}
	}

void LidarViewer::PointSelectorTool::glRenderActionTransparent(GLContextData& contextData) const
	{
	glPushAttrib(GL_COLOR_BUFFER_BIT|GL_ENABLE_BIT|GL_LINE_BIT|GL_POLYGON_BIT);
	glDisable(GL_LIGHTING);
	
	/* Draw the application's selection sphere at the main input device's position and with the current radius: */
	glPushMatrix();
	glMultMatrix(getButtonDeviceTransformation(0));
	glScale(radius);
	
	LidarViewer::DataItem* dataItem=contextData.retrieveDataItem<LidarViewer::DataItem>(application);
	glCallList(dataItem->influenceSphereDisplayListId);
	
	glPopMatrix();
	
	glPopAttrib();
	}

void LidarViewer::PointSelectorTool::update(void)
	{
	radius=application->defaultSelectorRadius;
	selectorMode=application->defaultSelectorMode;
	}
