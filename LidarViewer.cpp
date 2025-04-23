/***********************************************************************
LidarViewer - Viewer program for multiresolution LiDAR data.
Copyright (c) 2005-2025 Oliver Kreylos

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

#include "LidarViewer.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <stdexcept>
#include <Misc/SelfDestructPointer.h>
#include <Misc/FunctionCalls.h>
#include <Misc/StdError.h>
#include <Misc/PrintInteger.h>
#include <Misc/MessageLogger.h>
#include <Misc/CreateNumberedFileName.h>
#include <Misc/FileTests.h>
#include <Misc/StringMarshaller.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/ColorValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <IO/File.h>
#include <IO/ValueSource.h>
#include <IO/OpenFile.h>
#include <Cluster/MulticastPipe.h>
#include <Geometry/Vector.h>
#include <Geometry/AffineTransformation.h>
#include <Geometry/LinearUnit.h>
#include <GL/gl.h>
#include <GL/GLColorTemplates.h>
#include <GL/GLMaterialTemplates.h>
#include <GL/GLMaterial.h>
#include <GL/GLContextData.h>
#ifdef LIDARVIEWER_VISUALIZE_WATER
#include <GL/Extensions/GLARBVertexShader.h>
#include <GL/Extensions/GLARBFragmentShader.h>
#endif
#include <GL/GLValueCoders.h>
#include <GL/GLGeometryWrappers.h>
#include <GL/GLTransformationWrappers.h>
#include <GL/GLModels.h>
#include <GL/GLColorMap.h>
#include <GL/GLFrustum.h>
#include <GL/GLPrintError.h>
#include <GLMotif/StyleSheet.h>
#include <GLMotif/WidgetManager.h>
#include <GLMotif/PopupMenu.h>
#include <GLMotif/PopupWindow.h>
#include <GLMotif/RowColumn.h>
#include <GLMotif/Menu.h>
#include <GLMotif/Margin.h>
#include <GLMotif/Pager.h>
#include <GLMotif/Separator.h>
#include <GLMotif/Label.h>
#include <GLMotif/Button.h>
#include <GLMotif/CascadeButton.h>
#include <GLMotif/MaterialEditor.h>
#include <SceneGraph/NodeCreator.h>
#include <SceneGraph/VRMLFile.h>
#include <SceneGraph/TransformNode.h>
#include <Vrui/GlyphRenderer.h>
#include <Vrui/AffineCoordinateTransform.h>
#include <Vrui/Lightsource.h>
#include <Vrui/LightsourceManager.h>
#include <Vrui/Viewer.h>
#include <Vrui/CoordinateManager.h>
#include <Vrui/SceneGraphManager.h>
#include <Vrui/GenericAbstractToolFactory.h>
#include <Vrui/ToolManager.h>
#include <Vrui/ClusterSupport.h>
#include <Vrui/SurfaceNavigationTool.h>
#include <Vrui/SceneGraphSupport.h>
#if USE_COLLABORATION
#include <Collaboration2/DataType.icpp>
#include <Collaboration2/Client.h>
#endif

#include "Config.h"
#include "LidarOctree.h"
#include "ProjectorTool.h"
#include "PointSelectorTool.h"
#include "LidarSelectionSaver.h"
#include "Primitive.h"
#include "PointPrimitive.h"
#include "SpherePrimitive.h"
#include "LinePrimitive.h"
#include "CylinderPrimitive.h"
#include "PlanePrimitive.h"
#include "BruntonPrimitive.h"
#include "PrimitiveDraggerTool.h"
#include "PointClassifier.h"
#include "RidgeFinder.h"
#include "SceneGraph.h"
#include "LoadPointSet.h"
#include "FallingSphereProcessor.h"

/********************************************
Methods of class LidarViewer::RenderSettings:
********************************************/

LidarViewer::RenderSettings::RenderSettings(void)
	:pointBasedLighting(false),
	 surfaceMaterial(GLMaterial::Color(0.7f,0.7f,0.7f),GLMaterial::Color(0.5f,0.5f,0.5f),24.0f),
	 usePointColors(true),useSplatting(false),splatSize(0.1),
	 enableSun(false),sunAzimuth(180),sunElevation(45),
	 useTexturePlane(false),texturePlane(GPlane::Vector(0.0,0.0,1.0),0.0),texturePlaneScale(100.0),
	 #ifdef LIDARVIEWER_VISUALIZE_WATER
	 texturePlaneOffset(0.0),
	 #endif
	 planeDistanceExaggeration(1.0)
	{
	}

/**************************************
Methods of class LidarViewer::DataItem:
**************************************/

LidarViewer::DataItem::DataItem(GLContextData& contextData)
	:influenceSphereDisplayListId(glGenLists(1)),
	 pbls(contextData)
	 #ifdef LIDARVIEWER_VISUALIZE_WATER
	 ,waterShader(0)
	 #endif
	{
	glGenTextures(1,&planeColorMapTextureId);
	}

LidarViewer::DataItem::~DataItem(void)
	{
	glDeleteLists(influenceSphereDisplayListId,1);
	glDeleteTextures(1,&planeColorMapTextureId);
	
	#ifdef LIDARVIEWER_VISUALIZE_WATER
	glDeleteObjectARB(waterShader);
	#endif
	}

/****************************
Methods of class LidarViewer:
****************************/

void LidarViewer::changeSelectorModeCallback(GLMotif::RadioBox::ValueChangedCallbackData* cbData)
	{
	switch(cbData->radioBox->getToggleIndex(cbData->newSelectedToggle))
		{
		case 0:
			defaultSelectorMode=Add;
			break;
		
		case 1:
			defaultSelectorMode=Subtract;
			break;
		}
	
	/* Update the other selector mode radio box: */
	switch(defaultSelectorMode)
		{
		case Add:
			mainMenuSelectorModes->setSelectedToggle(0);
			interactionDialogSelectorModes->setSelectedToggle(0);
			break;
		
		case Subtract:
			mainMenuSelectorModes->setSelectedToggle(1);
			interactionDialogSelectorModes->setSelectedToggle(1);
			break;
		}
	
	if(overrideTools)
		{
		/* Apply the new setting to all currently existing point selector tools: */
		for(PointSelectorToolList::iterator pstIt=pointSelectorTools.begin();pstIt!=pointSelectorTools.end();++pstIt)
			(*pstIt)->update();
		}
	}

GLMotif::PopupMenu* LidarViewer::createSelectorModesMenu(void)
	{
	GLMotif::PopupMenu* selectorModesMenu=new GLMotif::PopupMenu("SelectorModesMenu",Vrui::getWidgetManager());
	
	GLMotif::RadioBox* selectorModes=new GLMotif::RadioBox("SelectorModes",selectorModesMenu,false);
	selectorModes->setSelectionMode(GLMotif::RadioBox::ALWAYS_ONE);
	
	selectorModes->addToggle("Add");
	selectorModes->addToggle("Subtract");
	
	selectorModes->manageChild();
	switch(defaultSelectorMode)
		{
		case Add:
			selectorModes->setSelectedToggle(0);
			break;
		
		case Subtract:
			selectorModes->setSelectedToggle(1);
			break;
		}
	selectorModes->getValueChangedCallbacks().add(this,&LidarViewer::changeSelectorModeCallback);
	
	mainMenuSelectorModes=selectorModes;
	
	selectorModesMenu->manageMenu();
	
	return selectorModesMenu;
	}

void LidarViewer::classifySelectionCallback(Misc::CallbackData* cbData)
	{
	for(int i=0;i<numOctrees;++i)
		{
		/* Create a point classification functor: */
		// PointClassifier pc(octrees[i],neighborhoodSize);
		RidgeFinder pc(octrees[i],neighborhoodSize);
		
		/* Classify all selected points: */
		octrees[i]->colorSelectedPoints(pc);
		}
	}

void LidarViewer::saveSelectionOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData)
	{
	bool success=false;
	
	if(Vrui::isHeadNode())
		{
		try
			{
			/* Create a selection saver functor: */
			LidarSelectionSaver lss(cbData->getSelectedPath().c_str(),offsets);
			if(octrees[0]->hasNormalVectors())
				{
				/* Save all selected points: */
				octrees[0]->processSelectedPointsWithNormals(lss);
				}
			else
				{
				/* Save all selected points: */
				octrees[0]->processSelectedPoints(lss);
				}
			
			if(Vrui::getMainPipe()!=0)
				{
				/* Send success flag to the cluster: */
				Vrui::getMainPipe()->write<unsigned char>(0);
				Vrui::getMainPipe()->flush();
				}
			
			success=true;
			}
		catch(const std::runtime_error& err)
			{
			if(Vrui::getMainPipe()!=0)
				{
				/* Send error flag and error message to the cluster: */
				Vrui::getMainPipe()->write<unsigned char>(1);
				Misc::writeCString(err.what(),*Vrui::getMainPipe());
				Vrui::getMainPipe()->flush();
				}
			
			/* Show an error message: */
			Misc::formattedUserError("Save Selection...: Could not write selection to file %s due to exception %s",cbData->getSelectedPath().c_str(),err.what());
			}
		}
	else
		{
		/* Read success code from the head node: */
		if(!(success=Vrui::getMainPipe()->read<unsigned char>()==0))
			{
			/* Read the error message and show an error dialog: */
			char* what=Misc::readCString(*Vrui::getMainPipe());
			Misc::formattedUserError("Save Selection...: Could not write selection to file %s due to exception %s",cbData->getSelectedPath().c_str(),what);
			delete[] what;
			}
		}
	
	if(success)
		{
		/* Remember the current directory for next time: */
		dataDirectory=cbData->selectedDirectory;
		}
	
	/* Destroy the file selection dialog: */
	cbData->fileSelectionDialog->close();
	}

void LidarViewer::saveSelectionCallback(Misc::CallbackData* cbData)
	{
	/* Create the default file name and file name filter: */
	const char* fileName;
	const char* filter;
	if(octrees[0]->hasNormalVectors())
		{
		fileName="SelectedPoints.xyzuvwrgb";
		filter=".xyzuvwrgb";
		}
	else
		{
		fileName="SelectedPoints.xyzrgb";
		filter=".xyzrgb";
		}
	
	try
		{
		/* Open the data directory if it doesn't already exist: */
		if(dataDirectory==0)
			dataDirectory=IO::openDirectory(".");
		
		/* Create a uniquely-named selection file in the data directory: */
		std::string selectionFileName=dataDirectory->createNumberedFileName(fileName,4);
		
		/* Create a file selection dialog to select an selection file name: */
		Misc::SelfDestructPointer<GLMotif::FileSelectionDialog> saveSelectionDialog(new GLMotif::FileSelectionDialog(Vrui::getWidgetManager(),"Save Selection...",dataDirectory,selectionFileName.c_str(),filter));
		saveSelectionDialog->getOKCallbacks().add(this,&LidarViewer::saveSelectionOKCallback);
		saveSelectionDialog->deleteOnCancel();
		
		/* Show the file selection dialog: */
		Vrui::popupPrimaryWidget(saveSelectionDialog.releaseTarget());
		}
	catch(const std::runtime_error& err)
		{
		/* Show an error message: */
		Misc::formattedUserError("Save Selection...: Could not save selection due to exception %s",err.what());
		}
	}

void LidarViewer::clearSelectionCallback(Misc::CallbackData* cbData)
	{
	/* Clear the point selection: */
	for(int i=0;i<numOctrees;++i)
		octrees[i]->clearSelection();
	}

GLMotif::PopupMenu* LidarViewer::createSelectionMenu(void)
	{
	GLMotif::PopupMenu* selectionMenu=new GLMotif::PopupMenu("SelectionMenu",Vrui::getWidgetManager());
	
	GLMotif::Button* classifySelectionButton=new GLMotif::Button("ClassifySelectionButton",selectionMenu,"Classify Selection");
	classifySelectionButton->getSelectCallbacks().add(this,&LidarViewer::classifySelectionCallback);
	
	GLMotif::Button* saveSelectionButton=new GLMotif::Button("SaveSelectionButton",selectionMenu,"Save Selection...");
	saveSelectionButton->getSelectCallbacks().add(this,&LidarViewer::saveSelectionCallback);
	
	new GLMotif::Separator("Separator1",selectionMenu,GLMotif::Separator::HORIZONTAL,0.0f,GLMotif::Separator::LOWERED);
	
	GLMotif::Button* clearSelectionButton=new GLMotif::Button("ClearSelectionButton",selectionMenu,"Clear Selection");
	clearSelectionButton->getSelectCallbacks().add(this,&LidarViewer::clearSelectionCallback);
	
	selectionMenu->manageMenu();
	
	return selectionMenu;
	}

GLMotif::PopupMenu* LidarViewer::createExtractionMenu(void)
	{
	GLMotif::PopupMenu* extractionMenu=new GLMotif::PopupMenu("ExtractionMenu",Vrui::getWidgetManager());
	
	GLMotif::Button* extractPlaneButton=new GLMotif::Button("ExtractPlaneButton",extractionMenu,"Extract Plane");
	extractPlaneButton->getSelectCallbacks().add(this,&LidarViewer::extractPlaneCallback);
	
	GLMotif::Button* extractBruntonButton=new GLMotif::Button("ExtractBruntonButton",extractionMenu,"Indicate Strike+Dip");
	extractBruntonButton->getSelectCallbacks().add(this,&LidarViewer::extractBruntonCallback);
	
	GLMotif::Button* extractLineButton=new GLMotif::Button("ExtractLineButton",extractionMenu,"Extract Line");
	extractLineButton->getSelectCallbacks().add(this,&LidarViewer::extractLineCallback);
	
	GLMotif::Button* extractSphereButton=new GLMotif::Button("ExtractSphereButton",extractionMenu,"Extract Sphere");
	extractSphereButton->getSelectCallbacks().add(this,&LidarViewer::extractSphereCallback);
	
	GLMotif::Button* extractCylinderButton=new GLMotif::Button("ExtractCylinderButton",extractionMenu,"Extract Cylinder");
	extractCylinderButton->getSelectCallbacks().add(this,&LidarViewer::extractCylinderCallback);
	
	GLMotif::Button* intersectPrimitivesButton=new GLMotif::Button("IntersectPrimitivesButton",extractionMenu,"Intersect Primitives");
	intersectPrimitivesButton->getSelectCallbacks().add(this,&LidarViewer::intersectPrimitivesCallback);
	
	GLMotif::Button* loadPrimitivesButton=new GLMotif::Button("LoadPrimitivesButton",extractionMenu,"Load Primitives...");
	loadPrimitivesButton->getSelectCallbacks().add(this,&LidarViewer::loadPrimitivesCallback);
	
	GLMotif::Button* savePrimitivesButton=new GLMotif::Button("SavePrimitivesButton",extractionMenu,"Save Primitives...");
	savePrimitivesButton->getSelectCallbacks().add(this,&LidarViewer::savePrimitivesCallback);
	
	new GLMotif::Separator("Separator1",extractionMenu,GLMotif::Separator::HORIZONTAL,0.0f,GLMotif::Separator::LOWERED);
	
	GLMotif::Button* deleteSelectedPrimitivesButton=new GLMotif::Button("DeleteSelectedPrimitivesButton",extractionMenu,"Delete Selected Primitives");
	deleteSelectedPrimitivesButton->getSelectCallbacks().add(this,&LidarViewer::deleteSelectedPrimitivesCallback);
	
	GLMotif::Button* clearPrimitivesButton=new GLMotif::Button("ClearPrimitivesButton",extractionMenu,"Clear Primitives");
	clearPrimitivesButton->getSelectCallbacks().add(this,&LidarViewer::clearPrimitivesCallback);
	
	extractionMenu->manageMenu();
	
	return extractionMenu;
	}

GLMotif::PopupMenu* LidarViewer::createDialogMenu(void)
	{
	GLMotif::PopupMenu* dialogMenu=new GLMotif::PopupMenu("DialogMenu",Vrui::getWidgetManager());
	
	if(numOctrees>1)
		{
		GLMotif::Button* showOctreeDialogButton=new GLMotif::Button("ShowOctreeDialogButton",dialogMenu,"Show Octree Dialog");
		showOctreeDialogButton->getSelectCallbacks().add(this,&LidarViewer::showOctreeDialogCallback);
		}
	
	GLMotif::Button* showRenderDialogButton=new GLMotif::Button("ShowRenderDialogButton",dialogMenu,"Show Render Dialog");
	showRenderDialogButton->getSelectCallbacks().add(this,&LidarViewer::showRenderDialogCallback);
	
	GLMotif::Button* showInteractionDialogButton=new GLMotif::Button("ShowInteractionDialogButton",dialogMenu,"Show Interaction Dialog");
	showInteractionDialogButton->getSelectCallbacks().add(this,&LidarViewer::showInteractionDialogCallback);
	
	dialogMenu->manageMenu();
	
	return dialogMenu;
	}

GLMotif::PopupMenu* LidarViewer::createMainMenu(void)
	{
	GLMotif::PopupMenu* mainMenu=new GLMotif::PopupMenu("MainMenu",Vrui::getWidgetManager());
	mainMenu->setTitle("LiDAR Viewer");
	
	GLMotif::CascadeButton* selectorModesCascade=new GLMotif::CascadeButton("SelectorModesCascade",mainMenu,"Selector Modes");
	selectorModesCascade->setPopup(createSelectorModesMenu());
	
	GLMotif::CascadeButton* selectionCascade=new GLMotif::CascadeButton("SelectionCascade",mainMenu,"Selection");
	selectionCascade->setPopup(createSelectionMenu());
	
	GLMotif::CascadeButton* extractionCascade=new GLMotif::CascadeButton("ExtractionCascade",mainMenu,"Primitives");
	extractionCascade->setPopup(createExtractionMenu());
	
	GLMotif::CascadeButton* dialogCascade=new GLMotif::CascadeButton("DialogCascade",mainMenu,"Dialogs");
	dialogCascade->setPopup(createDialogMenu());
	
	#if 0
	GLMotif::ToggleButton* updateTreeToggle=new GLMotif::ToggleButton("UpdateTreeToggle",mainMenu,"Update Tree");
	updateTreeToggle->setToggle(updateTree);
	updateTreeToggle->getValueChangedCallbacks().add(this,&LidarViewer::updateTreeCallback);
	#endif
	
	mainMenu->manageMenu();
	
	return mainMenu;
	}

GLMotif::PopupWindow* LidarViewer::createOctreeDialog(void)
	{
	GLMotif::PopupWindow* octreeDialog=new GLMotif::PopupWindow("OctreeDialog",Vrui::getWidgetManager(),"Octree Selection");
	octreeDialog->setCloseButton(true);
	octreeDialog->setResizableFlags(false,false);
	octreeDialog->popDownOnClose();
	
	GLMotif::RowColumn* octreeSelection=new GLMotif::RowColumn("OctreeSelection",octreeDialog,false);
	octreeSelection->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	octreeSelection->setPacking(GLMotif::RowColumn::PACK_GRID);
	octreeSelection->setNumMinorWidgets(1);
	
	char toggleName[20]="OctreeToggle";
	char toggleLabel[10];
	for(int i=0;i<numOctrees;++i)
		{
		char* tnPtr=Misc::print(i,toggleName+12+4);
		while(tnPtr>toggleName+12)
			*(--tnPtr)='0';
		GLMotif::ToggleButton* octreeToggle=new GLMotif::ToggleButton(toggleName,octreeSelection,Misc::print(i,toggleLabel+9));
		octreeToggle->setToggle(showOctrees[i]);
		octreeToggle->getValueChangedCallbacks().add(this,&LidarViewer::octreeSelectionCallback,i);
		}
	
	octreeSelection->manageChild();
	
	return octreeDialog;
	}

void LidarViewer::renderQualitySliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Get the new render quality: */
	renderQuality=Scalar(cbData->value);
	
	/* Set all octrees' render quality: */
	for(int i=0;i<numOctrees;++i)
		octrees[i]->setRenderQuality(renderQuality);
	}

void LidarViewer::backgroundColorSelectorCallback(GLMotif::HSVColorSelector::ValueChangedCallbackData* cbData)
	{
	/* Set Vrui's background color: */
	Vrui::setBackgroundColor(cbData->newColor);
	}

void LidarViewer::drawDistanceSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Set Vrui's backplane distance: */
	Vrui::setBackplaneDist(cbData->value*Vrui::getMeterFactor());
	}

void LidarViewer::renderSettingsChangedCallback(Misc::CallbackData* cbData)
	{
	#if USE_COLLABORATION
	if(koinonia!=0)
		{
		/* Share the new render settings with the server: */
		koinonia->replaceSharedObject(renderSettingsId);
		}
	#endif
	}

void LidarViewer::sunSettingsChangedCallback(Misc::CallbackData* cbData)
	{
	/* Update the sun light source: */
	updateSun();
	
	/* Call the render settings changed callback: */
	renderSettingsChangedCallback(cbData);
	}

void LidarViewer::distanceExaggerationSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Get new plane distance exaggeration factor: */
	renderSettings.planeDistanceExaggeration=cbData->value;
	
	/* Update the affine coordinate transformer to reflect the new exaggeration value: */
	Vrui::Vector tn=Vrui::Vector(renderSettings.texturePlane.getNormal());
	Vrui::Vector fTrans=tn*(Vrui::Scalar(renderSettings.texturePlane.getOffset())/tn.sqr());
	Vrui::Rotation fRot=Vrui::Rotation::rotateFromTo(Vrui::Vector(0,0,1),tn);
	Vrui::ATransform newTransform=Vrui::ATransform::translate(fTrans);
	newTransform*=Vrui::ATransform::rotate(fRot);
	newTransform*=Vrui::ATransform::scale(Vrui::ATransform::Scale(1.0,1.0,renderSettings.planeDistanceExaggeration));
	newTransform*=Vrui::ATransform::rotate(Geometry::invert(fRot));
	newTransform*=Vrui::ATransform::translate(-(fTrans+Vrui::Vector(offsets)));
	coordTransform->setTransform(newTransform);
	
	/* Call the render settings changed callback: */
	renderSettingsChangedCallback(cbData);
	}

#if USE_COLLABORATION

void LidarViewer::renderSettingsUpdatedCallback(KoinoniaClient* client,KoinoniaProtocol::ObjectID id,void* object,void* userData)
	{
	LidarViewer* thisPtr=static_cast<LidarViewer*>(userData);
	
	/* Update the sun light source: */
	thisPtr->updateSun();
	
	/* Update the affine coordinate transformer to reflect the new exaggeration value: */
	RenderSettings& rs=thisPtr->renderSettings;
	Vrui::Vector tn=Vrui::Vector(rs.texturePlane.getNormal());
	Vrui::Vector fTrans=tn*(Vrui::Scalar(rs.texturePlane.getOffset())/tn.sqr());
	Vrui::Rotation fRot=Vrui::Rotation::rotateFromTo(Vrui::Vector(0,0,1),tn);
	Vrui::ATransform newTransform=Vrui::ATransform::translate(fTrans);
	newTransform*=Vrui::ATransform::rotate(fRot);
	newTransform*=Vrui::ATransform::scale(Vrui::ATransform::Scale(1.0,1.0,rs.planeDistanceExaggeration));
	newTransform*=Vrui::ATransform::rotate(Geometry::invert(fRot));
	newTransform*=Vrui::ATransform::translate(-(fTrans+Vrui::Vector(thisPtr->offsets)));
	thisPtr->coordTransform->setTransform(newTransform);
	
	/* Update the UI: */
	thisPtr->renderDialog->updateVariables();
	}

void* LidarViewer::createPrimitiveFunction(KoinoniaClient* client,KoinoniaProtocol::NamespaceID namespaceId,DataType::TypeID type,void* userData)
	{
	/* Create a primitive object based on the given data type: */
	Primitive* result=0;
	if(type==PointPrimitive::getClassType())
		result=new PointPrimitive;
	else if(type==SpherePrimitive::getClassType())
		result=new SpherePrimitive;
	else if(type==LinePrimitive::getClassType())
		result=new LinePrimitive;
	else if(type==CylinderPrimitive::getClassType())
		result=new CylinderPrimitive;
	else if(type==PlanePrimitive::getClassType())
		result=new PlanePrimitive;
	else if(type==BruntonPrimitive::getClassType())
		result=new BruntonPrimitive;
	else
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Invalid primitive type %u",(unsigned int)(type));
	
	return result;
	}

void LidarViewer::primitiveCreatedCallback(KoinoniaClient* client,KoinoniaProtocol::NamespaceID namespaceId,KoinoniaProtocol::ObjectID objectId,void* object,void* userData)
	{
	/* Access the application object: */
	LidarViewer* thisPtr=static_cast<LidarViewer*>(userData);
	
	/* Set the new primitive's Koinonia ID: */
	Primitive* primitive=static_cast<Primitive*>(object);
	primitive->setObjectId(objectId);
	
	/* Check if the primitive is a Brunton: */
	if(dynamic_cast<BruntonPrimitive*>(primitive)!=0)
		{
		/* Create the Brunton visualization: */
		dynamic_cast<BruntonPrimitive*>(primitive)->buildBrunton();
		}
	
	/* Add the primitive: */
	thisPtr->addPrimitive(primitive);
	}

void LidarViewer::primitiveReplacedCallback(KoinoniaClient* client,KoinoniaProtocol::NamespaceID namespaceId,KoinoniaProtocol::ObjectID objectId,KoinoniaProtocol::VersionNumber newVersion,void* object,void* userData)
	{
	/* Invalidate the primitive's visualization: */
	static_cast<Primitive*>(object)->invalidate();
	}

void LidarViewer::primitiveDestroyedCallback(KoinoniaClient* client,KoinoniaProtocol::NamespaceID namespaceId,KoinoniaProtocol::ObjectID objectId,void* object,void* userData)
	{
	/* Access the application object: */
	LidarViewer* thisPtr=static_cast<LidarViewer*>(userData);
	
	/* Find the index of the deleted primitive: */
	Primitive* primitive=static_cast<Primitive*>(object);
	size_t primitiveIndex;
	for(primitiveIndex=0;primitiveIndex<thisPtr->primitives.size()&&thisPtr->primitives[primitiveIndex]!=primitive;++primitiveIndex)
		;
	if(primitiveIndex<thisPtr->primitives.size())
		{
		delete thisPtr->primitives[primitiveIndex];
		thisPtr->primitives.erase(thisPtr->primitives.begin()+primitiveIndex);
		thisPtr->primitiveSelectedFlags.erase(thisPtr->primitiveSelectedFlags.begin()+primitiveIndex);
		}
	}

#endif

GLMotif::PopupWindow* LidarViewer::createRenderDialog(void)
	{
	const GLMotif::StyleSheet& ss=*Vrui::getUiStyleSheet();
	
	GLMotif::PopupWindow* renderDialog=new GLMotif::PopupWindow("RenderDialog",Vrui::getWidgetManager(),"Render Settings");
	renderDialog->setCloseButton(true);
	renderDialog->setResizableFlags(true,false);
	renderDialog->popDownOnClose();
	
	GLMotif::Pager* renderPager=new GLMotif::Pager("RenderPager",renderDialog,false);
	
	/* Create a page with LOD settings: */
	renderPager->setNextPageName("LOD");
	
	GLMotif::RowColumn* lodBox=new GLMotif::RowColumn("LODBox",renderPager,false);
	lodBox->setOrientation(GLMotif::RowColumn::VERTICAL);
	lodBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	lodBox->setNumMinorWidgets(2);
	
	/* Create a slider/textfield combo to change the rendering quality: */
	new GLMotif::Label("RenderQualityLabel",lodBox,"Render Quality");
	
	GLMotif::TextFieldSlider* renderQualitySlider=new GLMotif::TextFieldSlider("RenderQualitySlider",lodBox,6,ss.fontHeight*10.0f);
	renderQualitySlider->getTextField()->setFloatFormat(GLMotif::TextField::FIXED);
	renderQualitySlider->getTextField()->setFieldWidth(5);
	renderQualitySlider->getTextField()->setPrecision(2);
	renderQualitySlider->setValueRange(-3.0,3.0,0.01);
	renderQualitySlider->getSlider()->addNotch(0.0f);
	renderQualitySlider->setValue(double(renderQuality));
	renderQualitySlider->getValueChangedCallbacks().add(this,&LidarViewer::renderQualitySliderCallback);
	
	/* Create a slider/textfield combo to change the focus+context weight: */
	new GLMotif::Label("FncWeightLabel",lodBox,"Focus + Context");
	
	GLMotif::TextFieldSlider* fncWeightSlider=new GLMotif::TextFieldSlider("FncWeightSlider",lodBox,6,ss.fontHeight*10.0f);
	fncWeightSlider->getTextField()->setFloatFormat(GLMotif::TextField::FIXED);
	fncWeightSlider->getTextField()->setFieldWidth(5);
	fncWeightSlider->getTextField()->setPrecision(2);
	fncWeightSlider->setValueRange(0.0,2.0,0.01);
	fncWeightSlider->track(fncWeight);
	
	/* Create a slider/textfield combo to change the point size: */
	new GLMotif::Label("PointSizeLabel",lodBox,"Point Size");
	
	GLMotif::TextFieldSlider* pointSizeSlider=new GLMotif::TextFieldSlider("PointSizeSlider",lodBox,6,ss.fontHeight*10.0f);
	pointSizeSlider->getTextField()->setFloatFormat(GLMotif::TextField::FIXED);
	pointSizeSlider->getTextField()->setFieldWidth(4);
	pointSizeSlider->getTextField()->setPrecision(1);
	pointSizeSlider->setValueRange(1.0,10.0,0.5);
	pointSizeSlider->track(pointSize);
	
	for(int i=0;i<3;++i)
		lodBox->setRowWeight(i,1.0f);
	
	lodBox->manageChild();
	
	/* Create a page with environment settings: */
	renderPager->setNextPageName("Environment");
	
	GLMotif::RowColumn* environmentBox=new GLMotif::RowColumn("EnvironmentBox",renderPager,false);
	environmentBox->setOrientation(GLMotif::RowColumn::VERTICAL);
	environmentBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	environmentBox->setNumMinorWidgets(2);
	
	/* Create a color selector to change the environment's background color: */
	new GLMotif::Label("BackgroundColorLabel",environmentBox,"Background Color");
	
	GLMotif::Margin* backgroundColorMargin=new GLMotif::Margin("BackgroundColorMargin",environmentBox,false);
	backgroundColorMargin->setAlignment(GLMotif::Alignment(GLMotif::Alignment::HCENTER));
	
	GLMotif::HSVColorSelector* backgroundColorSelector=new GLMotif::HSVColorSelector("BackgroundColorSelector",backgroundColorMargin);
	backgroundColorSelector->setCurrentColor(Vrui::getBackgroundColor());
	backgroundColorSelector->getValueChangedCallbacks().add(this,&LidarViewer::backgroundColorSelectorCallback);
	
	backgroundColorMargin->manageChild();
	
	/* Create a slider/textfield combo to change the backplane distance: */
	new GLMotif::Label("DrawDistanceLabel",environmentBox,"Draw Distance");
	
	GLMotif::TextFieldSlider* drawDistanceSlider=new GLMotif::TextFieldSlider("DrawDistanceSlider",environmentBox,10,ss.fontHeight*10.0f);
	drawDistanceSlider->getTextField()->setFloatFormat(GLMotif::TextField::SMART);
	drawDistanceSlider->getTextField()->setFieldWidth(8);
	drawDistanceSlider->getTextField()->setPrecision(8);
	drawDistanceSlider->setSliderMapping(GLMotif::TextFieldSlider::EXP10);
	double minDrawDist=Math::pow(10.0,Math::ceil(Math::log10(double(Vrui::getFrontplaneDist())*2.0/double(Vrui::getMeterFactor()))));
	drawDistanceSlider->setValueRange(minDrawDist,1000000.0,0.1);
	drawDistanceSlider->setValue(Vrui::getBackplaneDist()/Vrui::getMeterFactor());
	drawDistanceSlider->getValueChangedCallbacks().add(this,&LidarViewer::drawDistanceSliderCallback);
	
	/* Create a drop-down box to change the fog type: */
	new GLMotif::Label("FogTypeLabel",environmentBox,"Fog Type");
	
	environmentBox->manageChild();
	
	/* Check if any of the octrees have normal vectors: */
	bool haveNormalVectors=false;
	for(int i=0;i<numOctrees;++i)
		haveNormalVectors=haveNormalVectors||octrees[i]->hasNormalVectors();
	if(haveNormalVectors)
		{
		/* Create a page with lighting settings: */
		renderPager->setNextPageName("Lighting");
		
		GLMotif::RowColumn* lightingBox=new GLMotif::RowColumn("LightingBox",renderPager,false);
		lightingBox->setOrientation(GLMotif::RowColumn::VERTICAL);
		lightingBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
		lightingBox->setNumMinorWidgets(2);
		
		/* Create a toggle button to enable point-based lighting: */
		GLMotif::Margin* enableLightingMargin=new GLMotif::Margin("EnableLightingMargin",lightingBox,false);
		enableLightingMargin->setAlignment(GLMotif::Alignment(GLMotif::Alignment::LEFT,GLMotif::Alignment::VCENTER));
		
		GLMotif::ToggleButton* enableLightingToggle=new GLMotif::ToggleButton("EnableLightingToggle",enableLightingMargin,"Lighting");
		enableLightingToggle->setBorderWidth(0.0f);
		enableLightingToggle->setHAlignment(GLFont::Left);
		enableLightingToggle->track(renderSettings.pointBasedLighting);
		enableLightingToggle->getValueChangedCallbacks().add(this,&LidarViewer::renderSettingsChangedCallback);
		
		enableLightingMargin->manageChild();
		
		/* Create a toggle button to enable point colors: */
		GLMotif::Margin* usePointColorsMargin=new GLMotif::Margin("UsePointColorsMargin",lightingBox,false);
		usePointColorsMargin->setAlignment(GLMotif::Alignment(GLMotif::Alignment::LEFT,GLMotif::Alignment::VCENTER));
		
		GLMotif::ToggleButton* usePointColorsToggle=new GLMotif::ToggleButton("UsePointColorsToggle",usePointColorsMargin,"Use Point Colors");
		usePointColorsToggle->setBorderWidth(0.0f);
		usePointColorsToggle->setHAlignment(GLFont::Left);
		usePointColorsToggle->track(renderSettings.usePointColors);
		usePointColorsToggle->getValueChangedCallbacks().add(this,&LidarViewer::renderSettingsChangedCallback);
		
		usePointColorsMargin->manageChild();
		
		/* Create a button to enable point splatting: */
		GLMotif::Margin* useSplattingMargin=new GLMotif::Margin("UseSplattingMargin",lightingBox,false);
		useSplattingMargin->setAlignment(GLMotif::Alignment(GLMotif::Alignment::LEFT,GLMotif::Alignment::VCENTER));
		
		GLMotif::ToggleButton* useSplattingToggle=new GLMotif::ToggleButton("UseSplattingToggle",useSplattingMargin,"Splat Size");
		useSplattingToggle->setBorderWidth(0.0f);
		useSplattingToggle->setHAlignment(GLFont::Left);
		useSplattingToggle->track(renderSettings.useSplatting);
		useSplattingToggle->getValueChangedCallbacks().add(this,&LidarViewer::renderSettingsChangedCallback);
		
		useSplattingMargin->manageChild();
		
		GLMotif::TextFieldSlider* splatSizeSlider=new GLMotif::TextFieldSlider("SplatSizeSlider",lightingBox,6,ss.fontHeight*10.0f);
		splatSizeSlider->getTextField()->setFloatFormat(GLMotif::TextField::FIXED);
		splatSizeSlider->getTextField()->setFieldWidth(6);
		splatSizeSlider->getTextField()->setPrecision(3);
		splatSizeSlider->setSliderMapping(GLMotif::TextFieldSlider::EXP10);
		splatSizeSlider->setValueRange(0.001,1000.0,0.01);
		splatSizeSlider->track(renderSettings.splatSize);
		splatSizeSlider->getValueChangedCallbacks().add(this,&LidarViewer::renderSettingsChangedCallback);
		
		/* Create a toggle button to enable a fixed-position light source: */
		GLMotif::Margin* enableSunMargin=new GLMotif::Margin("EnableSunMargin",lightingBox,false);
		enableSunMargin->setAlignment(GLMotif::Alignment(GLMotif::Alignment::LEFT,GLMotif::Alignment::VCENTER));
		
		GLMotif::ToggleButton* enableSunToggle=new GLMotif::ToggleButton("SunToggle",enableSunMargin,"Sun Light Source");
		enableSunToggle->setBorderWidth(0.0f);
		enableSunToggle->setHAlignment(GLFont::Left);
		enableSunToggle->track(renderSettings.enableSun);
		enableSunToggle->getValueChangedCallbacks().add(this,&LidarViewer::sunSettingsChangedCallback);
		
		enableSunMargin->manageChild();
		
		/* Create a box with a pair of sliders to control the sun light source: */
		GLMotif::RowColumn* sunBox=new GLMotif::RowColumn("SunBox",lightingBox,false);
		sunBox->setOrientation(GLMotif::RowColumn::VERTICAL);
		sunBox->setNumMinorWidgets(2);
		
		new GLMotif::Label("SunAzimuthLabel",sunBox,"Azimuth");
		
		GLMotif::TextFieldSlider* sunAzimuthSlider=new GLMotif::TextFieldSlider("SunAzimuthSlider",sunBox,6,ss.fontHeight*10.0f);
		sunAzimuthSlider->getTextField()->setFloatFormat(GLMotif::TextField::FIXED);
		sunAzimuthSlider->getTextField()->setFieldWidth(3);
		sunAzimuthSlider->getTextField()->setPrecision(0);
		sunAzimuthSlider->setValueRange(0.0,360.0,1.0);
		sunAzimuthSlider->track(renderSettings.sunAzimuth);
		sunAzimuthSlider->getValueChangedCallbacks().add(this,&LidarViewer::sunSettingsChangedCallback);
		
		new GLMotif::Label("SunElevationLabel",sunBox,"Elevation");
		
		GLMotif::TextFieldSlider* sunElevationSlider=new GLMotif::TextFieldSlider("SunElevationSlider",sunBox,6,ss.fontHeight*10.0f);
		sunElevationSlider->getTextField()->setFloatFormat(GLMotif::TextField::FIXED);
		sunElevationSlider->getTextField()->setFieldWidth(2);
		sunElevationSlider->getTextField()->setPrecision(0);
		sunElevationSlider->setValueRange(-90.0,90.0,1.0);
		sunElevationSlider->track(renderSettings.sunElevation);
		sunElevationSlider->getValueChangedCallbacks().add(this,&LidarViewer::sunSettingsChangedCallback);
		
		sunBox->manageChild();
		
		for(int i=0;i<3;++i)
			lightingBox->setRowWeight(i,1.0f);
		
		lightingBox->manageChild();
		
		/* Create a page to edit material properties: */
		renderPager->setNextPageName("Material");
		
		GLMotif::Margin* materialMargin=new GLMotif::Margin("MaterialMargin",renderPager,false);
		materialMargin->setAlignment(GLMotif::Alignment(GLMotif::Alignment::HCENTER,GLMotif::Alignment::VCENTER));
		
		GLMotif::MaterialEditor* materialEditor=new GLMotif::MaterialEditor("MaterialEditor",materialMargin);
		materialEditor->track(renderSettings.surfaceMaterial);
		materialEditor->getValueChangedCallbacks().add(this,&LidarViewer::renderSettingsChangedCallback);
		
		materialMargin->manageChild();
		}
	
	/* Create a page with plane setting: */
	renderPager->setNextPageName("Plane");
	
	GLMotif::RowColumn* planeBox=new GLMotif::RowColumn("PlaneBox",renderPager,false);
	planeBox->setOrientation(GLMotif::RowColumn::VERTICAL);
	planeBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	planeBox->setNumMinorWidgets(2);
	
	new GLMotif::Label("ExaggerationLabel",planeBox,"Exaggeration");
	
	GLMotif::TextFieldSlider* exaggerationSlider=new GLMotif::TextFieldSlider("ExaggerationSlider",planeBox,8,ss.fontHeight*10.0f);
	exaggerationSlider->getTextField()->setFieldWidth(8);
	exaggerationSlider->getTextField()->setPrecision(3);
	exaggerationSlider->setSliderMapping(GLMotif::TextFieldSlider::EXP10);
	exaggerationSlider->setValueRange(0.05,20.0,0.02);
	exaggerationSlider->getSlider()->addNotch(0.0f);
	exaggerationSlider->track(renderSettings.planeDistanceExaggeration);
	exaggerationSlider->getValueChangedCallbacks().add(this,&LidarViewer::renderSettingsChangedCallback);
	
	/* Create a toggle button to enable plane distance visualization: */
	GLMotif::Margin* enableTexturePlaneMargin=new GLMotif::Margin("EnableTexturePlaneMargin",planeBox,false);
	enableTexturePlaneMargin->setAlignment(GLMotif::Alignment(GLMotif::Alignment::LEFT,GLMotif::Alignment::VCENTER));
	
	GLMotif::ToggleButton* enableTexturePlaneToggle=new GLMotif::ToggleButton("EnableTexturePlaneToggle",enableTexturePlaneMargin,"Show Plane Distance");
	enableTexturePlaneToggle->setBorderWidth(0.0f);
	enableTexturePlaneToggle->setHAlignment(GLFont::Left);
	enableTexturePlaneToggle->track(renderSettings.useTexturePlane);
	enableTexturePlaneToggle->getValueChangedCallbacks().add(this,&LidarViewer::renderSettingsChangedCallback);
	
	enableTexturePlaneMargin->manageChild();
	
	/* Create a slider to select the plane distance visualization scale: */
	GLMotif::TextFieldSlider* texturePlaneScaleSlider=new GLMotif::TextFieldSlider("TexturePlaneScaleSlider",planeBox,8,ss.fontHeight*10.0f);
	texturePlaneScaleSlider->getTextField()->setFieldWidth(8);
	texturePlaneScaleSlider->getTextField()->setPrecision(3);
	texturePlaneScaleSlider->setSliderMapping(GLMotif::TextFieldSlider::EXP10);
	texturePlaneScaleSlider->setValueRange(0.01,10000.0,0.1);
	texturePlaneScaleSlider->track(renderSettings.texturePlaneScale);
	texturePlaneScaleSlider->getValueChangedCallbacks().add(this,&LidarViewer::renderSettingsChangedCallback);
	
	#ifdef LIDARVIEWER_VISUALIZE_WATER
	
	/* Create a slider to adjust the vertical plane position: */
	new GLMotif::Label("PlaneOffsetLabel",planeBox,"Plane Offset");
	
	GLMotif::TextFieldSlider* texturePlaneOffsetSlider=new GLMotif::TextFieldSlider("TexturePlaneOffsetSlider",planeBox,8,ss.fontHeight*10.0f);
	texturePlaneOffsetSlider->getTextField()->setFieldWidth(8);
	texturePlaneOffsetSlider->getTextField()->setPrecision(3);
	texturePlaneOffsetSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	texturePlaneOffsetSlider->setValueRange(-20.0,20.0,0.01);
	texturePlaneOffsetSlider->getSlider()->addNotch(0.0f);
	texturePlaneOffsetSlider->track(renderSettings.texturePlaneOffset);
	texturePlaneOffsetSlider->getValueChangedCallbacks().add(this,&LidarViewer::renderSettingsChangedCallback);
	
	for(int i=0;i<3;++i)
		planeBox->setRowWeight(i,1.0f);
	
	#else
	
	for(int i=0;i<2;++i)
		planeBox->setRowWeight(i,1.0f);
	
	#endif
	
	planeBox->manageChild();
	
	renderPager->setCurrentChildIndex(0);
	renderPager->manageChild();
	
	return renderDialog;
	}

GLMotif::PopupWindow* LidarViewer::createInteractionDialog(void)
	{
	const GLMotif::StyleSheet& ss=*Vrui::getUiStyleSheet();
	
	GLMotif::PopupWindow* interactionDialog=new GLMotif::PopupWindow("InteractionDialog",Vrui::getWidgetManager(),"Interaction Settings");
	interactionDialog->setCloseButton(true);
	interactionDialog->setResizableFlags(true,false);
	interactionDialog->popDownOnClose();
	
	GLMotif::RowColumn* interactionSettings=new GLMotif::RowColumn("InteractionSettings",interactionDialog,false);
	interactionSettings->setOrientation(GLMotif::RowColumn::VERTICAL);
	interactionSettings->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	interactionSettings->setNumMinorWidgets(1);
	
	/* Create a box with tool settings: */
	GLMotif::Margin* toolSettingsMargin=new GLMotif::Margin("ToolSettingsMargin",interactionSettings,false);
	toolSettingsMargin->setAlignment(GLMotif::Alignment(GLMotif::Alignment::LEFT,GLMotif::Alignment::VCENTER));
	
	GLMotif::RowColumn* toolSettingsBox=new GLMotif::RowColumn("ToolSettingsBox",toolSettingsMargin,false);
	toolSettingsBox->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	toolSettingsBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	toolSettingsBox->setNumMinorWidgets(1);
	
	/* Create a toggle button to override existing tools' settings: */
	GLMotif::ToggleButton* overrideToolsToggle=new GLMotif::ToggleButton("OverrideToolsToggle",toolSettingsBox,"Override Tools");
	overrideToolsToggle->setBorderWidth(0.0f);
	overrideToolsToggle->setHAlignment(GLFont::Left);
	overrideToolsToggle->setToggle(overrideTools);
	overrideToolsToggle->getValueChangedCallbacks().add(this,&LidarViewer::overrideToolsCallback);
	
	new GLMotif::Separator("Separator1",toolSettingsBox,GLMotif::Separator::VERTICAL,0.0f,GLMotif::Separator::LOWERED);
	
	/* Create a radio box to select selection modes: */
	interactionDialogSelectorModes=new GLMotif::RadioBox("InteractionDialogSelectorModes",toolSettingsBox,false);
	interactionDialogSelectorModes->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	interactionDialogSelectorModes->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	interactionDialogSelectorModes->setSelectionMode(GLMotif::RadioBox::ALWAYS_ONE);
	
	interactionDialogSelectorModes->addToggle("Add");
	interactionDialogSelectorModes->addToggle("Subtract");
	
	switch(defaultSelectorMode)
		{
		case Add:
			interactionDialogSelectorModes->setSelectedToggle(0);
			break;
		
		case Subtract:
			interactionDialogSelectorModes->setSelectedToggle(1);
			break;
		}
	interactionDialogSelectorModes->getValueChangedCallbacks().add(this,&LidarViewer::changeSelectorModeCallback);
	interactionDialogSelectorModes->manageChild();
	
	toolSettingsBox->manageChild();
	toolSettingsMargin->manageChild();
	
	/* Create a box with sliders to adjust interaction sizes: */
	GLMotif::RowColumn* sliderBox=new GLMotif::RowColumn("SliderBox",interactionSettings,false);
	sliderBox->setOrientation(GLMotif::RowColumn::VERTICAL);
	sliderBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	sliderBox->setNumMinorWidgets(2);
	
	/* Create a slider to change the size of the selection brush: */
	new GLMotif::Label("BrushSizeLabel",sliderBox,"Brush Size");
	
	GLMotif::TextFieldSlider* brushSizeSlider=new GLMotif::TextFieldSlider("BrushSizeSlider",sliderBox,8,ss.fontHeight*10.0f);
	brushSizeSlider->getTextField()->setFieldWidth(7);
	brushSizeSlider->getTextField()->setPrecision(4);
	brushSizeSlider->setValueRange(double(defaultSelectorRadius)*0.1,double(defaultSelectorRadius)*5.0,double(defaultSelectorRadius)*0.01);
	brushSizeSlider->setValue(double(defaultSelectorRadius));
	brushSizeSlider->getValueChangedCallbacks().add(this,&LidarViewer::brushSizeSliderCallback);
	
	/* Create a slider to change the size of the processing neighborhood: */
	new GLMotif::Label("NeighborhoodSizeLabel",sliderBox,"Neighborhood Size");
	
	GLMotif::TextFieldSlider* neighborhoodSizeSlider=new GLMotif::TextFieldSlider("NeighborhoodSizeSlider",sliderBox,8,ss.fontHeight*10.0f);
	neighborhoodSizeSlider->getTextField()->setFieldWidth(7);
	neighborhoodSizeSlider->getTextField()->setPrecision(4);
	neighborhoodSizeSlider->setSliderMapping(GLMotif::TextFieldSlider::EXP10);
	neighborhoodSizeSlider->setValueRange(10.0e-3,10.0e3,0.1);
	neighborhoodSizeSlider->track(neighborhoodSize);
	
	sliderBox->manageChild();
	
	interactionSettings->manageChild();
	
	return interactionDialog;
	}

void LidarViewer::updateTexturePlane(const PlanePrimitive* plane)
	{
	/* Update the texture plane: */
	renderSettings.texturePlane=plane->getPlane();
	if(renderSettings.texturePlane.getNormal()[2]<0.0)
		renderSettings.texturePlane=GPlane(-renderSettings.texturePlane.getNormal(),-renderSettings.texturePlane.getOffset());
	renderSettings.texturePlane.normalize();
	
	#if USE_COLLABORATION
	if(koinonia!=0)
		{
		/* Share the new render settings with the server: */
		koinonia->replaceSharedObject(renderSettingsId);
		}
	#endif
	}

template <class PrimitiveParam>
inline PrimitiveParam* LidarViewer::extractPrimitive(void)
	{
	PrimitiveParam* primitive=0;
	if(Vrui::isHeadNode())
		{
		try
			{
			/* Extract a primitive: */
			primitive=new PrimitiveParam(octrees[0],Primitive::Vector(offsets));
			
			if(extractorPipe!=0)
				{
				/* Send the extracted primitive to the cluster: */
				extractorPipe->write(int(1));
				primitive->write(extractorPipe);
				extractorPipe->flush();
				}
			}
		catch(const std::runtime_error& err)
			{
			if(extractorPipe!=0)
				{
				/* Send an error message to the cluster: */
				extractorPipe->write(int(0));
				Misc::writeCString(err.what(),*extractorPipe);
				extractorPipe->flush();
				}
			
			/* Show an error message: */
			Misc::formattedUserError("LidarViewer: Unable to extract primitive due to exception %s",err.what());
			}
		}
	else
		{
		/* Read an error code from the head node: */
		if(extractorPipe->read<int>()!=0)
			{
			/* Read a primitive from the head node: */
			primitive=new PrimitiveParam(extractorPipe);
			}
		else
			{
			/* Read and show an error message: */
			std::string error=Misc::readCppString(*extractorPipe);
			Misc::formattedUserError("LidarViewer: Unable to extract primitive due to exception %s",error.c_str());
			}
		}
	
	if(primitive!=0)
		{
		/* Store the primitive: */
		lastPickedPrimitive=addPrimitive(primitive);
		
		#if USE_COLLABORATION
		if(koinonia!=0)
			{
			/* Share the primitive with the server: */
			primitive->setObjectId(koinonia->createNsObject(primitiveNamespaceId,primitive->getType(),primitive));
			}
		#endif
		}
	
	return primitive;
	}

int LidarViewer::addPrimitive(Primitive* newPrimitive)
	{
	/* Set the new primitive's color: */
	newPrimitive->setSurfaceColor(primitiveColor);
	newPrimitive->setGridColor(Primitive::Color(0.2f,0.2f,0.2f));
	
	/* Store the new primitive: */
	primitives.push_back(newPrimitive);
	primitiveSelectedFlags.push_back(false);
	return int(primitives.size())-1;
	}

Primitive::DragState* LidarViewer::pickPrimitive(const Primitive::Point& pickPos)
	{
	/* Pick against all primitives: */
	Primitive::Scalar maxDistance2(Math::sqr(Vrui::getPointPickDistance()));
	Primitive::DragState* dragState=0;
	int pickedPrimitiveIndex=-1;
	for(int i=0;i<int(primitives.size());++i)
		{
		Primitive::DragState* ds=primitives[i]->pick(pickPos,maxDistance2);
		if(ds!=0)
			{
			/* Remember the new drag state and the primitive that returned it: */
			delete dragState;
			dragState=ds;
			pickedPrimitiveIndex=i;
			}
		}
	
	/* Check if there was a successful pick: */
	if(pickedPrimitiveIndex>=0)
		{
		/* Remember the picked primitive: */
		lastPickedPrimitive=pickedPrimitiveIndex;
		}
	
	/* Return the drag state: */
	return dragState;
	}

void LidarViewer::dragPrimitive(Primitive::DragState* dragState,const Primitive::Point& dragPos)
	{
	/* Drag the picked primitive: */
	dragState->getPrimitive()->drag(dragState,dragPos);
	
	#if USE_COLLABORATION
	if(koinonia!=0)
		{
		/* Update the dragged primitive on the server: */
		koinonia->replaceNsObject(primitiveNamespaceId,dragState->getPrimitive()->getObjectId());
		}
	#endif
	}

void LidarViewer::togglePrimitive(Primitive* primitive)
	{
	/* Find the primitive in the list: */
	for(int i=0;i<int(primitives.size());++i)
		if(primitives[i]==primitive)
			{
			/* Toggle the primitive's selection state: */
			if(primitiveSelectedFlags[i])
				{
				/* Deselect the primitive: */
				primitiveSelectedFlags[i]=false;
				primitives[i]->setSurfaceColor(primitiveColor);
				}
			else
				{
				/* Select the primitive: */
				primitiveSelectedFlags[i]=true;
				primitives[i]->setSurfaceColor(selectedPrimitiveColor);
				}
			
			/* Stop searching: */
			break;
			}
	}

void LidarViewer::selectPrimitive(int primitiveIndex)
	{
	if(!primitiveSelectedFlags[primitiveIndex])
		{
		primitiveSelectedFlags[primitiveIndex]=true;
		primitives[primitiveIndex]->setSurfaceColor(primitiveColor);
		}
	}

void LidarViewer::deselectPrimitive(int primitiveIndex)
	{
	if(primitiveSelectedFlags[primitiveIndex])
		{
		primitiveSelectedFlags[primitiveIndex]=false;
		primitives[primitiveIndex]->setSurfaceColor(primitiveColor);
		}
	}

void LidarViewer::deletePrimitive(int primitiveIndex)
	{
	#if USE_COLLABORATION
	if(koinonia!=0)
		{
		/* Delete the primitive on the server: */
		koinonia->destroyNsObject(primitiveNamespaceId,primitives[primitiveIndex]->getObjectId());
		}
	#endif
	
	delete primitives[primitiveIndex];
	primitives.erase(primitives.begin()+primitiveIndex);
	primitiveSelectedFlags.erase(primitiveSelectedFlags.begin()+primitiveIndex);
	}

void LidarViewer::updateSun(void)
	{
	/* Check if the sun's state changed: */
	if(renderSettings.enableSun&&!sunEnabled)
		{
		/* Store the headlight enable states of all viewers and disable all headlights: */
		viewerHeadlightStates=new bool[Vrui::getNumViewers()];
		for(int i=0;i<Vrui::getNumViewers();++i)
			{
			viewerHeadlightStates[i]=Vrui::getViewer(i)->getHeadlight().isEnabled();
			Vrui::getViewer(i)->setHeadlightState(false);
			}
		
		/* Enable the sun light source: */
		sun->enable();
		}
	else if(!renderSettings.enableSun&&sunEnabled)
		{
		/* Reset the headlight enable states of all viewers: */
		for(int i=0;i<Vrui::getNumViewers();++i)
			Vrui::getViewer(i)->setHeadlightState(viewerHeadlightStates[i]);
		delete[] viewerHeadlightStates;
		viewerHeadlightStates=0;
		
		/* Disable the sun light source: */
		sun->disable();
		}
	sunEnabled=renderSettings.enableSun;
	
	if(sunEnabled)
		{
		/* Compute the light source's direction vector: */
		Vrui::Scalar z=Math::sin(Math::rad(renderSettings.sunElevation));
		Vrui::Scalar xy=Math::cos(Math::rad(renderSettings.sunElevation));
		Vrui::Scalar x=xy*Math::sin(Math::rad(renderSettings.sunAzimuth));
		Vrui::Scalar y=xy*Math::cos(Math::rad(renderSettings.sunAzimuth));
		sun->getLight().position=GLLight::Position(GLLight::Scalar(x),GLLight::Scalar(y),GLLight::Scalar(z),GLLight::Scalar(0));
		}
	}

LidarViewer::LidarViewer(int& argc,char**& argv)
	:Vrui::Application(argc,argv),
	 numOctrees(0),octrees(0),showOctrees(0),
	 coordTransform(0),
	 renderQuality(0),fncWeight(0.5),
	 pointSize(3.0f),
	 #if USE_COLLABORATION
	 koinonia(0),
	 #endif
	 viewerHeadlightStates(0),sun(0),sunEnabled(false),
	 updateTree(true),
	 lastFrameTime(Vrui::getApplicationTime()),
	 overrideTools(true),
	 defaultSelectorRadius(Vrui::getGlyphRenderer()->getGlyphSize()*Vrui::Scalar(2.5)),
	 brushColor(0.6f,0.6f,0.1f,0.5f),
	 defaultSelectorMode(Add),
	 neighborhoodSize(1),
	 extractorPipe(Vrui::openPipe()),
	 primitiveColor(0.5f,0.5f,0.1f,0.5f),
	 selectedPrimitiveColor(0.1f,0.5f,0.5f,0.5f),
	 lastPickedPrimitive(-1),
	 mainMenu(0),octreeDialog(0),renderDialog(0),interactionDialog(0),
	 dataDirectory(0)
	{
	memCacheSize=512;
	unsigned int gfxCacheSize=128;
	try
		{
		/* Open LidarViewer's configuration file: */
		Misc::ConfigurationFile configFile(LIDARVIEWER_CONFIGFILENAME);
		Misc::ConfigurationFileSection cfg=configFile.getSection("/LidarViewer");
		
		/* Override program settings from configuration file: */
		renderQuality=cfg.retrieveValue<Scalar>("./renderQuality",renderQuality);
		fncWeight=cfg.retrieveValue<Scalar>("./focusAndContextWeight",fncWeight);
		pointSize=cfg.retrieveValue<float>("./pointSize",pointSize);
		renderSettings.pointBasedLighting=cfg.retrieveValue<bool>("./enableLighting",renderSettings.pointBasedLighting);
		renderSettings.surfaceMaterial=cfg.retrieveValue<GLMaterial>("./surfaceMaterial",renderSettings.surfaceMaterial);
		renderSettings.usePointColors=cfg.retrieveValue<bool>("./usePointColors",renderSettings.usePointColors);
		renderSettings.useSplatting=cfg.retrieveValue<bool>("./useSplatting",renderSettings.useSplatting);
		renderSettings.splatSize=cfg.retrieveValue<double>("./splatSize",renderSettings.splatSize);
		renderSettings.enableSun=cfg.retrieveValue<bool>("./enableSun",renderSettings.enableSun);
		renderSettings.sunAzimuth=cfg.retrieveValue<Scalar>("./sunAzimuth",renderSettings.sunAzimuth);
		renderSettings.sunElevation=cfg.retrieveValue<Scalar>("./sunElevation",renderSettings.sunElevation);
		overrideTools=cfg.retrieveValue<bool>("./overrideTools",overrideTools);
		defaultSelectorRadius=cfg.retrieveValue<Vrui::Scalar>("./brushSize",defaultSelectorRadius);
		brushColor=cfg.retrieveValue<Misc::RGBA<Misc::Float32> >("./brushColor",brushColor);
		primitiveColor=cfg.retrieveValue<Primitive::Color>("./primitiveColor",primitiveColor);
		selectedPrimitiveColor=cfg.retrieveValue<Primitive::Color>("./selectedPrimitiveColor",selectedPrimitiveColor);
		memCacheSize=cfg.retrieveValue<unsigned int>("./memoryCacheSize",memCacheSize);
		gfxCacheSize=cfg.retrieveValue<unsigned int>("./graphicsCacheSize",gfxCacheSize);
		}
	catch(const std::runtime_error& err)
		{
		/* Just ignore the error */
		}
	
	/* Parse the command line: */
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"memoryCacheSize")==0)
				{
				if(i+1<argc)
					{
					++i;
					memCacheSize=atoi(argv[i]);
					}
				}
			else if(strcasecmp(argv[i]+1,"graphicsCacheSize")==0)
				{
				if(i+1<argc)
					{
					++i;
					gfxCacheSize=atoi(argv[i]);
					}
				}
			else if(strcasecmp(argv[i]+1,"renderQuality")==0)
				{
				if(i+1<argc)
					{
					++i;
					renderQuality=Scalar(atof(argv[i]));
					}
				}
			else if(strcasecmp(argv[i]+1,"focusAndContextWeight")==0)
				{
				if(i+1<argc)
					{
					++i;
					fncWeight=Scalar(atof(argv[i]));
					}
				}
			else if(strcasecmp(argv[i]+1,"pointSize")==0)
				{
				if(i+1<argc)
					{
					++i;
					pointSize=float(atof(argv[i]));
					}
				}
			else if(strcasecmp(argv[i]+1,"enableLighting")==0)
				renderSettings.pointBasedLighting=true;
			else if(strcasecmp(argv[i]+1,"usePointColors")==0)
				renderSettings.usePointColors=true;
			else if(strcasecmp(argv[i]+1,"sceneGraph")==0)
				{
				if(i+1<argc)
					{
					++i;
					
					try
						{
						/* Load the scene graph: */
						SceneGraph::GraphNodePointer sg=Vrui::getSceneGraphManager()->loadSceneGraph(argv[i]);
						
						/* Create the common root node if there is none yet: */
						if(sceneGraphRoot==0)
							sceneGraphRoot=new SceneGraph::TransformNode;
						
						/* Add the loaded scene graph to the list and the common root node: */
						sceneGraphRoot->addChild(*sg);
						sceneGraphs.push_back(sg);
						}
					catch(const std::runtime_error& err)
						{
						Misc::formattedUserWarning("Cannot load scene graph from file %s due to exception %s",argv[i],err.what());
						}
					}
				}
			}
		else
			{
			/* Load another LiDAR octree file: */
			lidarFileNames.push_back(argv[i]);
			LidarOctree** newOctrees=new LidarOctree*[numOctrees+1];
			for(int j=0;j<numOctrees;++j)
				newOctrees[j]=octrees[j];
			delete[] octrees;
			++numOctrees;
			octrees=newOctrees;
			
			/* Load a LiDAR octree file: */
			octrees[numOctrees-1]=new LidarOctree(argv[i],size_t(memCacheSize)*size_t(1024*1024),size_t(gfxCacheSize)*size_t(1024*1024));
			}
		}
	
	if(numOctrees==0)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"No octree file name provided");
	
	/* Initialize all octrees: */
	showOctrees=new bool[numOctrees];
	for(int i=0;i<numOctrees;++i)
		{
		octrees[i]->setRenderQuality(renderQuality);
		octrees[i]->setTreeUpdateFunction(treeUpdateNotificationCB,0);
		showOctrees[i]=true;
		}
	
	/* Check if all the octrees have the same linear unit: */
	Geometry::LinearUnit linearUnit;
	for(int i=0;i<numOctrees;++i)
		{
		/* Check if the LiDAR file contains a unit file: */
		std::string unitFileName=lidarFileNames[i];
		unitFileName.append("/Unit");
		if(Misc::isFileReadable(unitFileName.c_str()))
			{
			/* Read the unit file: */
			IO::ValueSource unit(IO::openFile(unitFileName.c_str()));
			unit.skipWs();
			Vrui::Scalar unitFactor=Vrui::Scalar(unit.readNumber());
			std::string unitName=unit.readString();
			
			/* Create a linear unit: */
			Geometry::LinearUnit fileLinearUnit(unitName.c_str(),unitFactor);
			if(linearUnit.unit==Geometry::LinearUnit::UNKNOWN)
				linearUnit=fileLinearUnit;
			else if(linearUnit.unit!=fileLinearUnit.unit||linearUnit.factor!=fileLinearUnit.factor)
				throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Octree file %s has mismatching units",lidarFileNames[i].c_str());
			}
		}
	
	/* Set the coordinate manager's linear unit: */
	Vrui::getCoordinateManager()->setUnit(linearUnit);
	
	/* Register a coordinate transform object to undo the coordinate offset done by the octree object and LiDAR preprocessor: */
	/* WARNING: This does not work properly for multiple octree files! */
	for(int i=0;i<3;++i)
		offsets[i]=double(octrees[0]->getPointOffset()[i]);
	std::string offsetFileName=lidarFileNames[0];
	offsetFileName.append("/Offset");
	if(Misc::isFileReadable(offsetFileName.c_str()))
		{
		/* Read the offset file: */
		IO::FilePtr offsetFile(IO::openFile(offsetFileName.c_str()));
		offsetFile->setEndianness(Misc::LittleEndian);
		for(int i=0;i<3;++i)
			offsets[i]-=offsetFile->read<double>();
		}
	Vrui::Vector offVec(offsets);
	coordTransform=new Vrui::AffineCoordinateTransform(Vrui::ATransform::translate(-offVec));
	Vrui::getCoordinateManager()->setCoordinateTransform(coordTransform);
	
	/* Check if there are any additional scene graphs: */
	if(sceneGraphRoot!=0)
		{
		/* Apply the coordinate transformation to the common root node: */
		sceneGraphRoot->translation.setValue(-offVec);
		sceneGraphRoot->update();
		
		/* Add the common root node to Vrui's navigational-space scene graph: */
		Vrui::getSceneGraphManager()->addNavigationalNode(*sceneGraphRoot);
		}
	
	/* Create the sun lightsource: */
	sun=Vrui::getLightsourceManager()->createLightsource(false);
	sun->disable();
	updateSun();
	
	/* Create the GUI: */
	mainMenu=createMainMenu();
	Vrui::setMainMenu(mainMenu);
	if(numOctrees>1)
		octreeDialog=createOctreeDialog();
	renderDialog=createRenderDialog();
	interactionDialog=createInteractionDialog();
	
	#if USE_COLLABORATION
	/* Check if there is a collaboration client: */
	Collab::Client* client=Collab::Client::getTheClient();
	if(client!=0)
		{
		/* Request a Koinonia client: */
		koinonia=KoinoniaClient::requestClient(client);
		
		/* Create a data type to represent the settings structure: */
		DataType renderSettingsTypes;
		DataType::TypeID materialScalarType=DataType::getAtomicType<GLMaterial::Scalar>();
		DataType::TypeID colorType=renderSettingsTypes.createFixedArray(4,materialScalarType);
		
		DataType::StructureElement materialElements[]=
			{
			{colorType,offsetof(GLMaterial,ambient)},
			{colorType,offsetof(GLMaterial,diffuse)},
			{colorType,offsetof(GLMaterial,specular)},
			{materialScalarType,offsetof(GLMaterial,shininess)},
			{colorType,offsetof(GLMaterial,emission)}
			};
		DataType::TypeID materialType=renderSettingsTypes.createStructure(5,materialElements,sizeof(GLMaterial));
		
		DataType::TypeID planeScalarType=DataType::getAtomicType<double>();
		
		DataType::StructureElement planeElements[]=
			{
			{renderSettingsTypes.createFixedArray(3,planeScalarType),0}, // offsetof(GPlane,normal)}, yikes
			{planeScalarType,3*sizeof(double)} // offsetof(GPlane,offset)} yikes
			};
		DataType::TypeID planeType=renderSettingsTypes.createStructure(2,planeElements,sizeof(GPlane));
		
		DataType::StructureElement renderSettingsElements[]=
			{
			{DataType::Bool,offsetof(RenderSettings,pointBasedLighting)},
			{materialType,offsetof(RenderSettings,surfaceMaterial)},
			{DataType::Bool,offsetof(RenderSettings,usePointColors)},
			{DataType::Bool,offsetof(RenderSettings,useSplatting)},
			{DataType::getAtomicType<double>(),offsetof(RenderSettings,splatSize)},
			{DataType::Bool,offsetof(RenderSettings,enableSun)},
			{DataType::getAtomicType<double>(),offsetof(RenderSettings,sunAzimuth)},
			{DataType::getAtomicType<double>(),offsetof(RenderSettings,sunElevation)},
			{DataType::Bool,offsetof(RenderSettings,useTexturePlane)},
			{planeType,offsetof(RenderSettings,texturePlane)},
			{DataType::getAtomicType<double>(),offsetof(RenderSettings,texturePlaneScale)},
			#ifdef LIDARVIEWER_VISUALIZE_WATER
			{DataType::getAtomicType<double>(),offsetof(RenderSettings,texturePlaneOffset)},
			#endif
			{DataType::getAtomicType<double>(),offsetof(RenderSettings,planeDistanceExaggeration)}
			};
		#ifdef LIDARVIEWER_VISUALIZE_WATER
		DataType::TypeID renderSettingsType=renderSettingsTypes.createStructure(13,renderSettingsElements,sizeof(RenderSettings));
		#else
		DataType::TypeID renderSettingsType=renderSettingsTypes.createStructure(12,renderSettingsElements,sizeof(RenderSettings));
		#endif
		
		/* Share the settings structure: */
		renderSettingsId=koinonia->shareObject("LidarViewer.renderSettings",(1U<<16)+0U,renderSettingsTypes,renderSettingsType,&renderSettings,&LidarViewer::renderSettingsUpdatedCallback,this);
		
		/* Create a data type dictionary to share extracted primitives: */
		Primitive::registerType(primitiveDataType);
		PointPrimitive::registerType(primitiveDataType);
		SpherePrimitive::registerType(primitiveDataType);
		LinePrimitive::registerType(primitiveDataType);
		CylinderPrimitive::registerType(primitiveDataType);
		PlanePrimitive::registerType(primitiveDataType);
		BruntonPrimitive::registerType(primitiveDataType);
		primitiveNamespaceId=koinonia->shareNamespace("LidarViewer.primitives",(1U<<16)+0U,primitiveDataType,
		                                              &LidarViewer::createPrimitiveFunction,this,
		                                              &LidarViewer::primitiveCreatedCallback,this,
		                                              &LidarViewer::primitiveReplacedCallback,this,
		                                              &LidarViewer::primitiveDestroyedCallback,this);
		}
	#endif
	
	/* Register the custom tool classes with the Vrui tool manager: */
	ProjectorTool::initClass();
	
	/* Create an abstract tool factory for LiDAR Viewer tools: */
	Vrui::ToolFactory* baseToolFactory=new Vrui::GenericAbstractToolFactory<PointSelectorTool>("LidarTool","LiDAR Viewer",0,*Vrui::getToolManager());
	Vrui::getToolManager()->addAbstractClass(baseToolFactory,Vrui::ToolManager::defaultToolFactoryDestructor);
	PointSelectorTool::initClass(baseToolFactory);
	PrimitiveDraggerTool::initClass(baseToolFactory);
	
	/* Initialize the scene graph: */
	createSceneGraph();
	}

LidarViewer::~LidarViewer(void)
	{
	/* Close the synchronization pipe: */
	delete extractorPipe;
	
	/* Destroy all primitives: */
	for(PrimitiveList::iterator pIt=primitives.begin();pIt!=primitives.end();++pIt)
		delete* pIt;
	
	/* Destroy the scene graph: */
	destroySceneGraph();
	
	/* Delete the GUI: */
	delete mainMenu;
	delete octreeDialog;
	delete renderDialog;
	delete interactionDialog;
	
	/* Delete the viewer headlight states: */
	delete[] viewerHeadlightStates;
	
	/* Delete all octrees: */
	for(int i=0;i<numOctrees;++i)
		delete octrees[i];
	delete[] octrees;
	delete[] showOctrees;
	}

void LidarViewer::toolCreationCallback(Vrui::ToolManager::ToolCreationCallbackData* cbData)
	{
	/* Let the base class at it first: */
	Vrui::Application::toolCreationCallback(cbData);
	
	/* Check if the new tool is a surface navigation tool: */
	Vrui::SurfaceNavigationTool* surfaceNavigationTool=dynamic_cast<Vrui::SurfaceNavigationTool*>(cbData->tool);
	if(surfaceNavigationTool!=0)
		{
		/* Set the new tool's alignment function: */
		surfaceNavigationTool->setAlignFunction(Misc::createFunctionCall(this,&LidarViewer::alignSurfaceFrame));
		}
	}

void LidarViewer::frame(void)
	{
	#if 0
	/* Get the current frame time and adapt the rendering quality: */
	double fr=Vrui::getCurrentFrameTime();
	if(fr>0.0)
		{
		fr=1.0/fr;
		if(fr<45.0)
			{
			renderQuality-=0.01f*(45.0-fr);
			octree->setRenderQuality(renderQuality);
			renderQualitySlider->setValue(double(renderQuality));
			}
		else if(fr>55.0)
			{
			renderQuality+=0.01f;
			octree->setRenderQuality(renderQuality);
			renderQualitySlider->setValue(double(renderQuality));
			}
		}
	#endif
	
	/* Prepare the next rendering pass: */
	Point displayCenter=Point(Vrui::getInverseNavigationTransformation().transform(Vrui::getDisplayCenter()));
	Scalar displaySize=Scalar(Vrui::getInverseNavigationTransformation().getScaling()*Vrui::getDisplaySize());
	for(int i=0;i<numOctrees;++i)
		if(showOctrees[i])
			{
			octrees[i]->startRenderPass();
			octrees[i]->setFocusAndContext(displayCenter,displaySize*Scalar(0.5),fncWeight);
			octrees[i]->setBaseSurfelSize(renderSettings.splatSize,float(Vrui::getNavigationTransformation().getScaling()));
			}
	}

void LidarViewer::display(GLContextData& contextData) const
	{
	/* Retrieve context entry: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	/* Set up basic OpenGL state: */
	glPushAttrib(GL_ENABLE_BIT|GL_LIGHTING_BIT|GL_LINE_BIT|GL_POINT_BIT|GL_TEXTURE_BIT);
	
	/* Enable fog: */
	glEnable(GL_FOG);
	glFogi(GL_FOG_DISTANCE_MODE_NV,GL_EYE_RADIAL_NV);
	glFogi(GL_FOG_MODE,GL_LINEAR);
	glFogf(GL_FOG_START,0.0f);
	glFogf(GL_FOG_END,GLfloat(Vrui::getBackplaneDist()));
	glFogfv(GL_FOG_COLOR,Vrui::getBackgroundColor().getRgba());
	
	if(renderSettings.pointBasedLighting&&octrees[0]->hasNormalVectors())
		{
		if(renderSettings.useTexturePlane)
			glMaterial(GLMaterialEnums::FRONT_AND_BACK,GLMaterial(GLMaterial::Color(1.0f,1.0f,1.0f),renderSettings.surfaceMaterial.specular,renderSettings.surfaceMaterial.shininess));
		else if(renderSettings.usePointColors)
			{
			glMaterial(GLMaterialEnums::FRONT_AND_BACK,renderSettings.surfaceMaterial);
			glEnable(GL_COLOR_MATERIAL);
			glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
			}
		else
			glMaterial(GLMaterialEnums::FRONT_AND_BACK,renderSettings.surfaceMaterial);
		
		/* Enable the point-based lighting shader: */
		dataItem->pbls.setUsePlaneDistance(renderSettings.useTexturePlane);
		dataItem->pbls.setUsePointColors(renderSettings.usePointColors);
		dataItem->pbls.setUseSplatting(renderSettings.useSplatting);
		dataItem->pbls.enable();
		
		#if 0
		
		/* Set the shader's eye coordinate surfel size: */
		dataItem->pbls.setSurfelSize(float(Vrui::getNavigationTransformation().getScaling()*renderSettings.splatSize));
		
		#endif
		
		if(renderSettings.useTexturePlane)
			{
			/* Set up distance plane texturing: */
			#ifdef LIDARVIEWER_VISUALIZE_WATER
			GPlane offsetTexturePlane=renderSettings.texturePlane;
			offsetTexturePlane.setOffset(offsetTexturePlane.getOffset()+renderSettings.texturePlaneOffset);
			dataItem->pbls.setDistancePlane(0,offsetTexturePlane,renderSettings.texturePlaneScale);
			#else
			dataItem->pbls.setDistancePlane(0,renderSettings.texturePlane,renderSettings.texturePlaneScale);
			#endif
			
			/* Bind the distance map texture: */
			glBindTexture(GL_TEXTURE_1D,dataItem->planeColorMapTextureId);
			}
		}
	else
		{
		/* Turn off lighting: */
		glDisable(GL_LIGHTING);
		
		if(renderSettings.useTexturePlane)
			{
			/* Set up automatic texture coordinate generation: */
			glTexGeni(GL_S,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
			GLdouble planeCoeff[4];
			for(int i=0;i<3;++i)
				planeCoeff[i]=renderSettings.texturePlane.getNormal()[i]/renderSettings.texturePlaneScale;
			#ifdef LIDARVIEWER_VISUALIZE_WATER
			planeCoeff[3]=0.5-(renderSettings.texturePlane.getOffset()+renderSettings.texturePlaneOffset)/renderSettings.texturePlaneScale;
			#else
			planeCoeff[3]=0.5-renderSettings.texturePlane.getOffset()/renderSettings.texturePlaneScale;
			#endif
			glTexGendv(GL_S,GL_OBJECT_PLANE,planeCoeff);
			glEnable(GL_TEXTURE_GEN_S);
			
			/* Enable 1D texture mapping: */
			glEnable(GL_TEXTURE_1D);
			glDisable(GL_TEXTURE_2D);
			glDisable(GL_TEXTURE_3D);
			glBindTexture(GL_TEXTURE_1D,dataItem->planeColorMapTextureId);
			glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);
			}
		}
	
	glPointSize(pointSize);
	
	if(renderSettings.planeDistanceExaggeration!=1.0)
		{
		/* Enable plane distance exaggeration: */
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		GPlane::Vector fTrans=renderSettings.texturePlane.getNormal()*(renderSettings.texturePlane.getOffset()/Geometry::sqr(renderSettings.texturePlane.getNormal()));
		Geometry::Rotation<GPlane::Scalar,3> fRot=Geometry::Rotation<GPlane::Scalar,3>::rotateFromTo(GPlane::Vector(0,0,1),renderSettings.texturePlane.getNormal());
		glTranslate(fTrans);
		glRotate(fRot);
		glScaled(1.0,1.0,renderSettings.planeDistanceExaggeration);
		glRotate(Geometry::invert(fRot));
		glTranslate(-fTrans);
		}
	
	/* Render the LiDAR point tree: */
	LidarOctree::Frustum frustum;
	frustum.setFromGL();
	for(int i=0;i<numOctrees;++i)
		if(showOctrees[i])
			octrees[i]->glRenderAction(frustum,dataItem->pbls,contextData);
	
	if(renderSettings.planeDistanceExaggeration!=1.0)
		glPopMatrix();
	
	/* Reset OpenGL state: */
	if(renderSettings.pointBasedLighting&&octrees[0]->hasNormalVectors())
		{
		/* Disable the point-based lighting shader: */
		dataItem->pbls.disable();
		
		if(renderSettings.useTexturePlane)
			glBindTexture(GL_TEXTURE_1D,0);
		}
	else
		{
		if(renderSettings.useTexturePlane)
			{
			/* Disable 1D texture mapping: */
			glBindTexture(GL_TEXTURE_1D,0);
			glDisable(GL_TEXTURE_1D);
			
			/* Disable automatic texture coordinate generation: */
			glDisable(GL_TEXTURE_GEN_S);
			}
		}
	
	#ifdef LIDARVIEWER_VISUALIZE_WATER
	#if 0
	
	if(renderSettings.useTexturePlane)
		{
		/* Draw the texture plane itself as a transparent quad: */
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		glDepthMask(GL_FALSE);
		glDisable(GL_CULL_FACE);
		
		GPlane::Point center=renderSettings.texturePlane.project(octrees[0]->getDomainCenter());
		GPlane::Vector z=renderSettings.texturePlane.getNormal();
		z.normalize();
		center+=z*renderSettings.texturePlaneOffset;
		GPlane::Vector x=Geometry::normal(renderSettings.texturePlane.getNormal());
		x*=octrees[0]->getDomainRadius()/Geometry::mag(x);
		GPlane::Vector y=Geometry::cross(renderSettings.texturePlane.getNormal(),x);
		y*=octrees[0]->getDomainRadius()/Geometry::mag(x);
		
		glUseProgramObjectARB(dataItem->waterShader);
		
		glBegin(GL_QUADS);
		glNormal(renderSettings.texturePlane.getNormal());
		glTexCoord2f(-8192.0f,-8192.0f);
		glVertex(center-x-y);
		glTexCoord2f(8192.0f,-8192.0f);
		glVertex(center+x-y);
		glTexCoord2f(8192.0f,8192.0f);
		glVertex(center+x+y);
		glTexCoord2f(-8192.0f,8192.0f);
		glVertex(center-x+y);
		glEnd();
		
		glUseProgramObjectARB(0);
		
		glEnable(GL_CULL_FACE);
		
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);
		}
	
	#endif
	#endif
	
	glPopAttrib();
	
	glPushAttrib(GL_ENABLE_BIT|GL_LIGHTING_BIT|GL_LINE_BIT|GL_POINT_BIT|GL_POLYGON_BIT|GL_TEXTURE_BIT);
	
	/* Render LiDAR Viewer's own scene graph: */
	renderSceneGraph(contextData);
	
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);
	
	/* Render all extracted primitives: */
	for(PrimitiveList::const_iterator pIt=primitives.begin();pIt!=primitives.end();++pIt)
		(*pIt)->glRenderAction(contextData);
	
	glPopAttrib();
	}

void LidarViewer::resetNavigation(void)
	{
	/* Initialize the navigation transformation: */
	Vrui::setNavigationTransformation(octrees[0]->getDomainCenter(),octrees[0]->getDomainRadius(),Vrui::Vector(0,0,1));
	}

void LidarViewer::glRenderActionTransparent(GLContextData& contextData) const
	{
	glPushAttrib(GL_COLOR_BUFFER_BIT|GL_ENABLE_BIT|GL_LINE_BIT|GL_POINT_BIT|GL_POLYGON_BIT);
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);
	
	/* Go to navigational space: */
	Vrui::goToNavigationalSpace(contextData);
	
	/* Render all extracted primitives: */
	for(PrimitiveList::const_iterator pIt=primitives.begin();pIt!=primitives.end();++pIt)
		(*pIt)->glRenderActionTransparent(contextData);
	
	/* Go back to physical space: */
	glPopMatrix();
	
	glPopAttrib();
	}

void LidarViewer::initContext(GLContextData& contextData) const
	{
	/* Create a new context entry: */
	DataItem* dataItem=new DataItem(contextData);
	contextData.addDataItem(this,dataItem);
	
	/* Create the influence sphere display list: */
	glNewList(dataItem->influenceSphereDisplayListId,GL_COMPILE);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glColor<4>(brushColor.getComponents());
	glDrawSphereIcosahedron(1.0,5);
	glBlendFunc(GL_ONE,GL_ONE);
	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
	glLineWidth(1.0f);
	glColor3f(0.025f,0.025f,0.025f);
	glDrawSphereIcosahedron(1.0,5);
	glEndList();
	
	/* Create the texture plane color map: */
	const int numPlaneColors=7;
	static const GLColor<GLfloat,3> planeColors[numPlaneColors]=
		{
		GLColor<GLfloat,3>(0.0f,0.0f,0.5f),
		GLColor<GLfloat,3>(0.0f,0.0f,1.0f),
		GLColor<GLfloat,3>(0.0f,1.0f,1.0f),
		GLColor<GLfloat,3>(1.0f,1.0f,1.0f),
		GLColor<GLfloat,3>(1.0f,1.0f,0.0f),
		GLColor<GLfloat,3>(1.0f,0.0f,0.0f),
		GLColor<GLfloat,3>(0.5f,0.0f,0.0f)
		};
	GLColor<GLfloat,3>* planeColorMap=new GLColor<GLfloat,3>[1024];
	for(int i=0;i<1024;++i)
		{
		int ci0=(i*(numPlaneColors-1))/1023;
		int ci1=ci0<numPlaneColors-1?ci0+1:numPlaneColors-1;
		float cd=float(i*(numPlaneColors-1)-ci0*1023)/1023.0f;
		for(int j=0;j<3;++j)
			planeColorMap[i][j]=planeColors[ci0][j]*(1.0f-cd)+planeColors[ci1][j]*cd;
		}
	
	#if 0
	
	/* Add notches to the color map: */
	for(int i=0;i<=20;++i)
		{
		planeColorMap[(i*1023+10)/20]=GLColor<GLfloat,3>(0.0f,0.0f,0.0f);
		}
	
	#endif
	
	/* Create the color map texture image: */
	glBindTexture(GL_TEXTURE_1D,dataItem->planeColorMapTextureId);
	glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
	glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
	glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_BASE_LEVEL,0);
	glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_MAX_LEVEL,0);
	glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
	glTexImage1D(GL_TEXTURE_1D,0,GL_RGB,1024,0,GL_RGB,GL_FLOAT,planeColorMap);
	glBindTexture(GL_TEXTURE_1D,0);
	
	delete[] planeColorMap;
	
	#ifdef LIDARVIEWER_VISUALIZE_WATER
	
	/* Create a shader to texture the water surface: */
	GLhandleARB waterVertexShader=glCompileVertexShaderFromFile("WaterShader.vs");
	GLhandleARB waterFragmentShader=glCompileFragmentShaderFromFile("WaterShader.fs");
	dataItem->waterShader=glLinkShader(waterVertexShader,waterFragmentShader);
	
	#endif
	}

void LidarViewer::alignSurfaceFrame(Vrui::SurfaceNavigationTool::AlignmentData& alignmentData)
	{
	/* Get the frame's base point: */
	Vrui::Point base=alignmentData.surfaceFrame.getOrigin();
	
	Scalar surfaceZ=base[2];
	if(Vrui::isHeadNode())
		{
		/* Drop a sphere onto the LiDAR point cloud: */
		base[2]+=alignmentData.probeSize+alignmentData.maxClimb;
		FallingSphereProcessor fsp(base,alignmentData.probeSize);
		for(int i=0;i<numOctrees;++i)
			octrees[i]->processPointsInBox(fsp.getBox(),fsp);
		
		/* Get the surface elevation: */
		if(fsp.getMinZ()!=Math::Constants<Scalar>::min)
			surfaceZ=fsp.getMinZ()-alignmentData.probeSize;
		
		if(Vrui::getMainPipe()!=0)
			{
			/* Send the surface elevation to the cluster: */
			Vrui::getMainPipe()->write<Scalar>(surfaceZ);
			}
		}
	else
		{
		/* Read the surface elevation from the head node: */
		Vrui::getMainPipe()->read<Scalar>(surfaceZ);
		}
	
	/* Move the frame's base point: */
	base[2]=surfaceZ;
	
	/* Align the frame with the terrain's x and y directions: */
	alignmentData.surfaceFrame=Vrui::NavTransform(base-Vrui::Point::origin,Vrui::Rotation::identity,alignmentData.surfaceFrame.getScaling());
	}

void LidarViewer::extractPlaneCallback(Misc::CallbackData* cbData)
	{
	/* Extract a plane primitive and update the texture plane if successful: */
	PlanePrimitive* primitive=extractPrimitive<PlanePrimitive>();
	updateTexturePlane(primitive);
	}

void LidarViewer::extractBruntonCallback(Misc::CallbackData* cbData)
	{
	/* Extract a brunton primitive and update the texture plane if successful: */
	BruntonPrimitive* primitive=extractPrimitive<BruntonPrimitive>();
	updateTexturePlane(primitive);
	}

void LidarViewer::extractLineCallback(Misc::CallbackData* cbData)
	{
	/* Extract a line primitive: */
	extractPrimitive<LinePrimitive>();
	}

void LidarViewer::extractSphereCallback(Misc::CallbackData* cbData)
	{
	/* Extract a sphere primitive: */
	extractPrimitive<SpherePrimitive>();
	}

void LidarViewer::extractCylinderCallback(Misc::CallbackData* cbData)
	{
	/* Extract a cylinder primitive: */
	extractPrimitive<CylinderPrimitive>();
	}

void LidarViewer::intersectPrimitivesCallback(Misc::CallbackData* cbData)
	{
	/* Sort all selected primitives by their base types: */
	std::vector<PlanePrimitive*> planes;
	std::vector<LinePrimitive*> lines;
	std::vector<PointPrimitive*> points;
	for(unsigned int i=0;i<primitives.size();++i)
		if(primitiveSelectedFlags[i])
			{
			if(dynamic_cast<PlanePrimitive*>(primitives[i])!=0)
				planes.push_back(dynamic_cast<PlanePrimitive*>(primitives[i]));
			else if(dynamic_cast<LinePrimitive*>(primitives[i])!=0)
				lines.push_back(dynamic_cast<LinePrimitive*>(primitives[i]));
			else if(dynamic_cast<PointPrimitive*>(primitives[i])!=0)
				points.push_back(dynamic_cast<PointPrimitive*>(primitives[i]));
			}
	
	Primitive* primitive=0;
	if(Vrui::isHeadNode())
		{
		try
			{
			/* Intersect a primitive: */
			if(planes.size()==2&&lines.empty()&&points.empty())
				{
				/* Create a line by intersecting two planes: */
				primitive=new LinePrimitive(planes.data(),Primitive::Vector(offsets));
				}
			else if(planes.size()==3&&lines.empty()&&points.empty())
				{
				/* Create a point by intersecting three planes: */
				primitive=new PointPrimitive(planes.data(),Primitive::Vector(offsets));
				}
			else if(planes.size()==1&&lines.size()==1&&points.empty())
				{
				/* Create a point by intersecting a plane and a line: */
				primitive=new PointPrimitive(planes[0],lines[0],Primitive::Vector(offsets));
				}
			else
				throw std::runtime_error("mismatching selected primitives");
			
			if(extractorPipe!=0)
				{
				/* Send the extracted primitive to the cluster: */
				extractorPipe->write(int(1));
				primitive->write(extractorPipe);
				extractorPipe->flush();
				}
			}
		catch(const std::runtime_error& err)
			{
			if(extractorPipe!=0)
				{
				/* Send an error message to the cluster: */
				extractorPipe->write(int(0));
				Misc::writeCString(err.what(),*extractorPipe);
				extractorPipe->flush();
				}
			
			/* Show an error message: */
			Misc::formattedUserError("LidarViewer: Unable to intersect primitives due to exception %s",err.what());
			}
		}
	else
		{
		/* Read an error code from the head node: */
		if(extractorPipe->read<int>()!=0)
			{
			/* Read a primitive from the head node: */
			if(planes.size()==2&&lines.empty()&&points.empty())
				primitive=new LinePrimitive(extractorPipe);
			else if(planes.size()==3&&lines.empty()&&points.empty())
				primitive=new PointPrimitive(extractorPipe);
			else if(planes.size()==1&&lines.size()==1&&points.empty())
				primitive=new PointPrimitive(extractorPipe);
			}
		else
			{
			/* Read and show an error message: */
			std::string error=Misc::readCppString(*extractorPipe);
			Misc::formattedUserError("LidarViewer: Unable to intersect primitives due to exception %s",error.c_str());
			}
		}
	
	if(primitive!=0)
		{
		/* Store the primitive: */
		lastPickedPrimitive=addPrimitive(primitive);
		
		#if USE_COLLABORATION
		if(koinonia!=0)
			{
			/* Share the primitive with the server: */
			primitive->setObjectId(koinonia->createNsObject(primitiveNamespaceId,primitive->getType(),primitive));
			}
		#endif
		
		/* Unselect all primitives: */
		for(int i=0;i<int(primitives.size());++i)
			deselectPrimitive(i);
		}
	}

void LidarViewer::loadPrimitivesOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData)
	{
	try
		{
		/* Open the primitive file: */
		IO::FilePtr primitiveFile(cbData->selectedDirectory->openFile(cbData->selectedFileName));
		primitiveFile->setEndianness(Misc::LittleEndian);
		
		/* Read the file header: */
		char header[40];
		primitiveFile->read(header,sizeof(header));
		if(strcmp(header,"LidarViewer primitive file v1.3       \n")!=0)
			throw std::runtime_error("Not a valid version 1.3 primitive file");
		
		/* Read all primitives in the file: */
		while(!primitiveFile->eof())
			{
			/* Read the primitive type: */
			int primitiveType=primitiveFile->read<int>();
			
			/* Create a primitive of the appropriate type: */
			Primitive* newPrimitive=0;
			switch(primitiveType)
				{
				case 0:
					newPrimitive=new PointPrimitive(*primitiveFile,-Primitive::Vector(offsets));
					break;
				
				case 1:
					newPrimitive=new SpherePrimitive(*primitiveFile,-Primitive::Vector(offsets));
					break;
				
				case 2:
					newPrimitive=new LinePrimitive(*primitiveFile,-Primitive::Vector(offsets));
					break;
				
				case 3:
					newPrimitive=new CylinderPrimitive(*primitiveFile,-Primitive::Vector(offsets));
					break;
				
				case 4:
					{
					PlanePrimitive* newPlane=new PlanePrimitive(*primitiveFile,-Primitive::Vector(offsets));
					updateTexturePlane(newPlane);
					newPrimitive=newPlane;
					
					break;
					}
				
				case 5:
					{
					BruntonPrimitive* newBrunton=new BruntonPrimitive(*primitiveFile,-Primitive::Vector(offsets));
					updateTexturePlane(newBrunton);
					newPrimitive=newBrunton;
					
					break;
					}
				
				default:
					throw Misc::makeStdErr(0,"Unknown primitive type %d",primitiveType);
				}
			
			/* Store the primitive: */
			lastPickedPrimitive=addPrimitive(newPrimitive);
			
			#if USE_COLLABORATION
			if(koinonia!=0)
				{
				/* Share the primitive with the server: */
				newPrimitive->setObjectId(koinonia->createNsObject(primitiveNamespaceId,newPrimitive->getType(),newPrimitive));
				}
			#endif
			}
		
		/* Remember the current directory for next time: */
		dataDirectory=cbData->selectedDirectory;
		}
	catch(const std::runtime_error& err)
		{
		/* Show an error message: */
		Misc::formattedUserError("Load Primitives...: Could not load primitives from file %s due to exception %s",cbData->getSelectedPath().c_str(),err.what());
		}
	
	/* Close the file selection dialog: */
	cbData->fileSelectionDialog->close();
	}

void LidarViewer::loadPrimitivesCallback(Misc::CallbackData* cbData)
	{
	try
		{
		/* Open the data directory if it doesn't already exist: */
		if(dataDirectory==0)
			dataDirectory=IO::openDirectory(".");
		
		/* Create a file selection dialog to select a primitive file: */
		Misc::SelfDestructPointer<GLMotif::FileSelectionDialog> loadPrimitivesDialog(new GLMotif::FileSelectionDialog(Vrui::getWidgetManager(),"Load Primitives...",dataDirectory,".dat"));
		loadPrimitivesDialog->getOKCallbacks().add(this,&LidarViewer::loadPrimitivesOKCallback);
		loadPrimitivesDialog->deleteOnCancel();
		
		/* Show the file selection dialog: */
		Vrui::popupPrimaryWidget(loadPrimitivesDialog.releaseTarget());
		}
	catch(const std::runtime_error& err)
		{
		/* Show an error message: */
		Misc::formattedUserError("Load Primitives...: Could not load primitives due to exception %s",err.what());
		}
	}

void LidarViewer::savePrimitivesOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData)
	{
	try
		{
		/* Open the primitive file: */
		IO::FilePtr primitiveFile(cbData->selectedDirectory->openFile(cbData->selectedFileName,IO::File::WriteOnly));
		primitiveFile->setEndianness(Misc::LittleEndian);
		
		/* Write the file header: */
		char header[40];
		snprintf(header,sizeof(header),"LidarViewer primitive file v1.3       \n");
		primitiveFile->write(header,sizeof(header));
		
		/* Write all primitives to the file: */
		for(PrimitiveList::const_iterator pIt=primitives.begin();pIt!=primitives.end();++pIt)
			{
			/* Determine and write the primitive type: */
			if(dynamic_cast<const PointPrimitive*>(*pIt)!=0)
				{
				if(dynamic_cast<const SpherePrimitive*>(*pIt)!=0)
					primitiveFile->write<int>(1);
				else
					primitiveFile->write<int>(0);
				}
			else if(dynamic_cast<const LinePrimitive*>(*pIt)!=0)
				{
				if(dynamic_cast<const CylinderPrimitive*>(*pIt)!=0)
					primitiveFile->write<int>(3);
				else
					primitiveFile->write<int>(2);
				}
			else if(dynamic_cast<const PlanePrimitive*>(*pIt)!=0)
				{
				if(dynamic_cast<const BruntonPrimitive*>(*pIt)!=0)
					primitiveFile->write<int>(5);
				else
					primitiveFile->write<int>(4);
				}
			
			/* Write the primitive: */
			(*pIt)->write(*primitiveFile,Primitive::Vector(offsets));
			}
		}
	catch(const std::runtime_error& err)
		{
		/* Show an error message: */
		Misc::formattedUserError("Save Primitives...: Could not write primitives to file %s due to exception %s",cbData->getSelectedPath().c_str(),err.what());
		}
	
	/* Close the file selection dialog: */
	cbData->fileSelectionDialog->close();
	}

void LidarViewer::savePrimitivesCallback(Misc::CallbackData* cbData)
	{
	try
		{
		/* Open the data directory if it doesn't already exist: */
		if(dataDirectory==0)
			dataDirectory=IO::openDirectory(".");

		/* Create a uniquely-named primitive file name in the data directory: */
		std::string primitiveFileName=dataDirectory->createNumberedFileName("SavedPrimitives.dat",4);

		/* Create a file selection dialog to select a primitive file: */
		Misc::SelfDestructPointer<GLMotif::FileSelectionDialog> savePrimitivesDialog(new GLMotif::FileSelectionDialog(Vrui::getWidgetManager(),"Save Primitives...",dataDirectory,primitiveFileName.c_str(),".dat"));
		savePrimitivesDialog->getOKCallbacks().add(this,&LidarViewer::savePrimitivesOKCallback);
		savePrimitivesDialog->deleteOnCancel();
		
		/* Show the file selection dialog: */
		Vrui::popupPrimaryWidget(savePrimitivesDialog.releaseTarget());
		}
	catch(const std::runtime_error& err)
		{
		/* Show an error message: */
		Misc::formattedUserError("Save Primitives...: Could not save primitives due to exception %s",err.what());
		}
	}

void LidarViewer::deleteSelectedPrimitivesCallback(Misc::CallbackData* cbData)
	{
	for(int i=primitives.size()-1;i>=0;--i)
		if(primitiveSelectedFlags[i])
			deletePrimitive(i);
	lastPickedPrimitive=-1;
	}

void LidarViewer::clearPrimitivesCallback(Misc::CallbackData* cbData)
	{
	/* Destroy all primitives: */
	for(PrimitiveList::iterator pIt=primitives.begin();pIt!=primitives.end();++pIt)
		{
		#if USE_COLLABORATION
		if(koinonia!=0)
			{
			/* Delete the primitive on the server: */
			koinonia->destroyNsObject(primitiveNamespaceId,(*pIt)->getObjectId());
			}
		#endif
		
		delete *pIt;
		}
	primitives.clear();
	primitiveSelectedFlags.clear();
	lastPickedPrimitive=-1;
	}

void LidarViewer::showOctreeDialogCallback(Misc::CallbackData* cbData)
	{
	/* Open the octree selection dialog: */
	Vrui::popupPrimaryWidget(octreeDialog);
	}

void LidarViewer::octreeSelectionCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData,const int& octreeIndex)
	{
	/* Update the octree visibility flag: */
	showOctrees[octreeIndex]=cbData->set;
	}

void LidarViewer::showRenderDialogCallback(Misc::CallbackData* cbData)
	{
	/* Open the render dialog: */
	Vrui::popupPrimaryWidget(renderDialog);
	}

void LidarViewer::showInteractionDialogCallback(Misc::CallbackData* cbData)
	{
	/* Open the interaction dialog: */
	Vrui::popupPrimaryWidget(interactionDialog);
	}

void LidarViewer::overrideToolsCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	overrideTools=cbData->set;
	if(overrideTools)
		{
		/* Apply current tool settings to all existing tools: */
		for(PointSelectorToolList::iterator pstIt=pointSelectorTools.begin();pstIt!=pointSelectorTools.end();++pstIt)
			(*pstIt)->update();
		}
	}

void LidarViewer::brushSizeSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Get the new brush size: */
	defaultSelectorRadius=cbData->value;
	
	if(overrideTools)
		{
		/* Apply current tool settings to all existing tools: */
		for(PointSelectorToolList::iterator pstIt=pointSelectorTools.begin();pstIt!=pointSelectorTools.end();++pstIt)
			(*pstIt)->update();
		}
	}

void LidarViewer::updateTreeCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	updateTree=cbData->set;
	}

VRUI_APPLICATION_RUN(LidarViewer)
