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

#ifndef LIDARVIEWER_INCLUDED
#define LIDARVIEWER_INCLUDED

#include "Config.h"

#include <vector>
#include <Misc/SizedTypes.h>
#include <Misc/RGBA.h>
#include <Misc/CallbackData.h>
#include <IO/Directory.h>
#include <Geometry/Plane.h>
#include <GL/gl.h>
#include <GL/GLMaterial.h>
#include <GL/GLObject.h>
#ifdef LIDARVIEWER_VISUALIZE_WATER
#include <GL/Extensions/GLARBShaderObjects.h>
#endif
#include <GLMotif/RadioBox.h>
#include <GLMotif/ToggleButton.h>
#include <GLMotif/TextFieldSlider.h>
#include <GLMotif/HSVColorSelector.h>
#include <GLMotif/FileSelectionDialog.h>
#include <SceneGraph/DOGTransformNode.h>
#include <SceneGraph/SceneGraphList.h>
#include <Vrui/Vrui.h>
#include <Vrui/Application.h>
#include <Vrui/SurfaceNavigationTool.h>
#include <Vrui/TransparentObject.h>
#if USE_COLLABORATION
#include <Collaboration2/DataType.h>
#include <Collaboration2/Plugins/KoinoniaClient.h>
#endif

#include "LidarTypes.h"
#include "Primitive.h"
#include "PointBasedLightingShader.h"

/* Forward declarations: */
namespace Cluster {
class MulticastPipe;
}
namespace GLMotif {
class PopupMenu;
class PopupWindow;
}
namespace Vrui {
class InputDevice;
class Lightsource;
class AffineCoordinateTransform;
}

class LidarOctree;
class PlanePrimitive;

/* Namespace shortcuts: */
#if USE_COLLABORATION
using Collab::Plugins::KoinoniaClient;
#endif

class LidarViewer:public Vrui::Application,public Vrui::TransparentObject,public GLObject
	{
	/* Embedded classes: */
	private:
	typedef Geometry::Plane<double,3> GPlane;
	
	enum SelectorMode // Enumerated type for selection modes
		{
		Add,Subtract
		};
	
	class ProjectorTool; // Class for transformer tools projecting a ray-based input device onto a LiDAR point cloud
	class PointSelectorTool; // Class for tools to select/deselect points
	class PrimitiveDraggerTool; // Class for tools to select/deselect and drag primitives
	class ProfileExtractorTool; // Class for tools to extract profiles from a LiDAR point cloud
	
	friend class ProjectorTool;
	friend class PointSelectorTool;
	friend class PrimitiveDraggerTool;
	friend class ProfileExtractorTool;
	
	typedef std::vector<PointSelectorTool*> PointSelectorToolList;
	typedef std::vector<Primitive*> PrimitiveList;
	
	struct RenderSettings // Structure holding environment-independent rendering settings
		{
		/* Elements: */
		public:
		bool pointBasedLighting; // Flag whether points are rendered with illumination
		GLMaterial surfaceMaterial; // Surface material properties used during illuminated rendering
		bool usePointColors; // Flag whether to use points' colors during illuminated rendering
		bool useSplatting; // Flag whether to use point splats when illumination is enabled
		double splatSize; // Size of point splats in model coordinate units
		bool enableSun; // Flag whether to use a sun light source instead of all viewer's headlights
		double sunAzimuth,sunElevation; // Azimuth and elevation angles of sun light source in degrees
		bool useTexturePlane; // Flag whether to use automatically generated texture coordinates to visualize point distance from a plane
		GPlane texturePlane; // Plane equation of the texture-generating plane
		double texturePlaneScale; // Scale factor for texture plane distances
		#ifdef LIDARVIEWER_VISUALIZE_WATER
		double texturePlaneOffset; // Additional offset for texture plane distances
		#endif
		double planeDistanceExaggeration; // Exaggeration factor for distances orthogonal to the texture plane
		
		/* Constructors and destructors: */
		RenderSettings(void); // Creates default rendering settings
		};
	
	struct DataItem:public GLObject::DataItem
		{
		/* Elements: */
		public:
		GLuint influenceSphereDisplayListId; // ID of display list to render transparent spheres
		GLuint planeColorMapTextureId; // Texture object ID of texture plane color map
		PointBasedLightingShader pbls; // Shader for point-based lighting
		#ifdef LIDARVIEWER_VISUALIZE_WATER
		GLhandleARB waterShader; // Shader to generate a water-like texture on-the-fly
		#endif
		
		/* Constructors and destructors: */
		DataItem(GLContextData& contextData);
		virtual ~DataItem(void);
		};
	
	/* Elements: */
	unsigned int memCacheSize; // Memory cache size for the LiDAR data representation in MB
	std::vector<std::string> lidarFileNames; // Array of file names from with the octrees were loaded
	int numOctrees; // Number of LiDAR octrees rendered in parallel
	LidarOctree** octrees; // Array of LiDAR data representations rendered in parallel
	bool* showOctrees; // Array of flags to disable individual LiDAR data representations
	double offsets[3]; // Coordinate offsets that need to be added to points stored in the octree(s) to reconstruct original point positions
	Vrui::AffineCoordinateTransform* coordTransform; // Pointer to a coordinate transformation to undo point offsets and handle vertical exaggeration
	Scalar renderQuality; // The current rendering quality (adapted to achieve optimal frame rate)
	Scalar fncWeight; // Weight factor for focus+context LOD adjustment
	float pointSize; // The pixel size used to render LiDAR points
	RenderSettings renderSettings; // Environment-independent rendering settings
	SceneGraph::DOGTransformNodePointer sceneGraphRoot; // Common root node for additional scene graphs
	SceneGraph::SceneGraphList sceneGraphList; // Dynamic list of additionally loaded scene graphs
	#if USE_COLLABORATION
	KoinoniaClient* koinonia; // Koinonia plug-in protocol
	KoinoniaProtocol::ObjectID renderSettingsId; // Koinonia ID to share rendering settings
	DataType primitiveDataType; // Data type dictionary to share extracted primitives
	KoinoniaProtocol::NamespaceID primitiveNamespaceId; // ID for the namespace to share extracted primitives
	#endif
	bool* viewerHeadlightStates; // Enable states of all viewers' headlights at the last time the sun light source was turned on
	Vrui::Lightsource* sun; // Light source representing the sun
	bool sunEnabled; // Flag whether the sun light source is currently enabled
	bool updateTree; // Flag if the tree is continuously updated
	double lastFrameTime; // Application time of last frame; used to calculate render performance
	
	/* Interaction state: */
	bool overrideTools; // Flag whether interaction settings changes influence existing tools
	Vrui::Scalar defaultSelectorRadius; // Default physical-coordinate size for new interaction brushes
	Misc::RGBA<Misc::Float32> brushColor; // Color to render selection brush, with transparency
	SelectorMode defaultSelectorMode; // Selection mode for new selector locators
	PointSelectorToolList pointSelectorTools; // List of currently existing point selector tools
	Scalar neighborhoodSize; // Size of neighborhood for point classification
	Cluster::MulticastPipe* extractorPipe; // Pipe to synchronize feature extraction on a distributed rendering cluster
	Primitive::Color primitiveColor; // Color to render primitives, with transparency
	Primitive::Color selectedPrimitiveColor; // Color to render selected primitives, with transparency
	PrimitiveList primitives; // List of extracted primitives
	int lastPickedPrimitive; // Index of the most recently picked primitive
	std::vector<bool> primitiveSelectedFlags; // List of selected flags for extracted primitives
	
	/* Vrui state: */
	GLMotif::PopupMenu* mainMenu; // The program's main menu
	GLMotif::RadioBox* mainMenuSelectorModes;
	GLMotif::PopupWindow* octreeDialog; // The dialog to select individual octrees
	GLMotif::PopupWindow* renderDialog; // The rendering settings dialog
	GLMotif::PopupWindow* interactionDialog; // The interaction settings dialog
	GLMotif::RadioBox* interactionDialogSelectorModes;
	IO::DirectoryPtr dataDirectory; // Last directory from/to which selections or primitives were loaded/saved
	
	/* Private methods: */
	void changeSelectorModeCallback(GLMotif::RadioBox::ValueChangedCallbackData* cbData);
	GLMotif::PopupMenu* createSelectorModesMenu(void);
	void classifySelectionCallback(Misc::CallbackData* cbData);
	void saveSelectionOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData);
	void saveSelectionCallback(Misc::CallbackData* cbData);
	void clearSelectionCallback(Misc::CallbackData* cbData);
	GLMotif::PopupMenu* createSelectionMenu(void);
	GLMotif::PopupMenu* createExtractionMenu(void);
	GLMotif::PopupMenu* createDialogMenu(void);
	GLMotif::PopupMenu* createMainMenu(void);
	GLMotif::PopupWindow* createOctreeDialog(void);
	void renderQualitySliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void backgroundColorSelectorCallback(GLMotif::HSVColorSelector::ValueChangedCallbackData* cbData);
	void drawDistanceSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void renderSettingsChangedCallback(Misc::CallbackData* cbData);
	void sunSettingsChangedCallback(Misc::CallbackData* cbData);
	void distanceExaggerationSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	#if USE_COLLABORATION
	static void renderSettingsUpdatedCallback(KoinoniaClient* client,KoinoniaProtocol::ObjectID id,void* object,void* userData);
	static void* createPrimitiveFunction(KoinoniaClient* client,KoinoniaProtocol::NamespaceID namespaceId,DataType::TypeID type,void* userData);
	static void primitiveCreatedCallback(KoinoniaClient* client,KoinoniaProtocol::NamespaceID namespaceId,KoinoniaProtocol::ObjectID objectId,void* object,void* userData);
	static void primitiveReplacedCallback(KoinoniaClient* client,KoinoniaProtocol::NamespaceID namespaceId,KoinoniaProtocol::ObjectID objectId,KoinoniaProtocol::VersionNumber newVersion,void* object,void* userData);
	static void primitiveDestroyedCallback(KoinoniaClient* client,KoinoniaProtocol::NamespaceID namespaceId,KoinoniaProtocol::ObjectID objectId,void* object,void* userData);
	#endif
	GLMotif::PopupWindow* createRenderDialog(void);
	GLMotif::PopupWindow* createInteractionDialog(void);
	static void treeUpdateNotificationCB(void* userData)
		{
		Vrui::requestUpdate();
		};
	void updateTexturePlane(const PlanePrimitive* plane); // Updates the texture generation plane based on the given plane primitive
	template <class PrimitiveParam>
	PrimitiveParam* extractPrimitive(void); // Extracts a primitive of some type from the octree
	int addPrimitive(Primitive* newPrimitive); // Adds a primitive to the list; returns the index of the newly added primitive
	Primitive::DragState* pickPrimitive(const Primitive::Point& pickPos); // Picks the list of extracted primitives and returns a pick result, or null if there was no valid picl
	void dragPrimitive(Primitive::DragState* dragState,const Primitive::Point& dragPos); // Drags the primitive associated with the given pick result
	void togglePrimitive(Primitive* primitive); // Toggles the selection state of the given primitive
	void selectPrimitive(int primitiveIndex); // Selects the given primitive
	void deselectPrimitive(int primitiveIndex); // Deselects the given primitive
	void deletePrimitive(int primitiveIndex); // Deletes the given primitive from the list
	void updateSun(void); // Updates the state of the sun light source
	
	/* Constructors and destructors: */
	public:
	LidarViewer(int& argc,char**& argv);
	virtual ~LidarViewer(void);
	
	/* Methods from class Vrui::Application: */
	virtual void toolCreationCallback(Vrui::ToolManager::ToolCreationCallbackData* cbData);
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	virtual void resetNavigation(void);
	
	/* Methods from class Vrui::TransparentObject: */
	virtual void glRenderActionTransparent(GLContextData& contextData) const;
	
	/* Methods from class GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	
	/* New methods: */
	void alignSurfaceFrame(Vrui::SurfaceNavigationTool::AlignmentData& alignmentData);
	void extractPlaneCallback(Misc::CallbackData* cbData);
	void extractBruntonCallback(Misc::CallbackData* cbData);
	void extractLineCallback(Misc::CallbackData* cbData);
	void extractSphereCallback(Misc::CallbackData* cbData);
	void extractCylinderCallback(Misc::CallbackData* cbData);
	void intersectPrimitivesCallback(Misc::CallbackData* cbData);
	void loadPrimitivesOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData);
	void loadPrimitivesCallback(Misc::CallbackData* cbData);
	void savePrimitivesOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData);
	void savePrimitivesCallback(Misc::CallbackData* cbData);
	void deleteSelectedPrimitivesCallback(Misc::CallbackData* cbData);
	void clearPrimitivesCallback(Misc::CallbackData* cbData);
	void showOctreeDialogCallback(Misc::CallbackData* cbData);
	void octreeSelectionCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData,const int& octreeIndex);
	void showRenderDialogCallback(Misc::CallbackData* cbData);
	void showInteractionDialogCallback(Misc::CallbackData* cbData);
	void showSceneGraphListCallback(Misc::CallbackData* cbData);
	void overrideToolsCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	void brushSizeSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void updateTreeCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	};

#endif
