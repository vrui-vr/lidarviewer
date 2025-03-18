########################################################################
# Makefile for LiDAR Viewer, a visualization and analysis application
# for large 3D point cloud data.
# Copyright (c) 2004-2025 Oliver Kreylos
#
# This file is part of the WhyTools Build Environment.
# 
# The WhyTools Build Environment is free software; you can redistribute
# it and/or modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2 of the
# License, or (at your option) any later version.
# 
# The WhyTools Build Environment is distributed in the hope that it will
# be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with the WhyTools Build Environment; if not, write to the Free
# Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
# 02111-1307 USA
########################################################################

# Directory containing the Vrui build system. The directory below
# matches the default Vrui installation; if Vrui's installation
# directory was changed during Vrui's installation, the directory below
# must be adapted.
VRUI_MAKEDIR := /usr/local/share/Vrui-13.0/make

# Base installation directory for LiDAR Viewer. If this is set to the
# default of $(PWD), LiDAR Viewer does not have to be installed to be
# run. Created executables and resources will be installed in the bin
# and share directories under the given base directory, respectively.
# Important note: Do not use ~ as an abbreviation for the user's home
# directory here; use $(HOME) instead.
INSTALLDIR := $(shell pwd)

########################################################################
# Everything below here should not have to be changed
########################################################################

# Name of the package
PROJECT_NAME = LidarViewer
PROJECT_DISPLAYNAME = LiDAR Viewer

# Version number for installation subdirectories. This is used to keep
# subsequent release versions of LiDAR Viewer from clobbering each
# other. The value should be identical to the major.minor version
# number found in VERSION in the root package directory.
PROJECT_MAJOR = 2
PROJECT_MINOR = 20

# Include definitions for the system environment and system-provided
# packages
include $(VRUI_MAKEDIR)/SystemDefinitions
include $(VRUI_MAKEDIR)/Packages.System
include $(VRUI_MAKEDIR)/Configuration.Vrui
include $(VRUI_MAKEDIR)/Packages.Vrui

# Check if the Vrui Collaboration Infrastructure is installed
-include $(VRUI_MAKEDIR)/Configuration.Collaboration
-include $(VRUI_MAKEDIR)/Packages.Collaboration
ifdef COLLABORATION_VERSION
  HAVE_COLLABORATION = 1
else
  HAVE_COLLABORATION = 0
endif

########################################################################
# Specify additional compiler and linker flags
########################################################################

CFLAGS += -Wall -pedantic

########################################################################
# List common packages used by all components of this project
# (Supported packages can be found in $(VRUI_MAKEDIR)/Packages.*)
########################################################################

PACKAGES = MYGEOMETRY MYMATH MYIO MYMISC

########################################################################
# Specify all final targets
########################################################################

CONFIGS = 
EXECUTABLES =
PLUGINS =

CONFIGS += Config.h

EXECUTABLES += $(EXEDIR)/CalcLasRange \
               $(EXEDIR)/LidarPreprocessor \
               $(EXEDIR)/LidarSpotRemover \
               $(EXEDIR)/LidarSubtractor \
               $(EXEDIR)/LidarIlluminator \
               $(EXEDIR)/LidarColorMapper \
               $(EXEDIR)/PaulBunyan \
               $(EXEDIR)/LidarExporter \
               $(EXEDIR)/LidarGridder \
               $(EXEDIR)/LidarViewer \
               $(EXEDIR)/PointSetSimilarity \
               $(EXEDIR)/PrintPrimitiveFile

ALL = $(EXECUTABLES) $(PLUGINS)

.PHONY: all
all: $(CONFIGS) $(ALL)

########################################################################
# Pseudo-target to print configuration options and configure the package
########################################################################

.PHONY: config config-invalidate
config: config-invalidate $(DEPDIR)/config

config-invalidate:
	@mkdir -p $(DEPDIR)
	@touch $(DEPDIR)/Configure-Begin

$(DEPDIR)/Configure-Begin:
	@mkdir -p $(DEPDIR)
	@echo "---- $(PROJECT_FULLDISPLAYNAME) configuration options: ----"
ifdef COLLABORATION_VERSION
	@echo "Collaborative visualization enabled"
else
	@echo "Collaborative visualization disabled"
endif
	@touch $(DEPDIR)/Configure-Begin

$(DEPDIR)/Configure-LidarViewer: $(DEPDIR)/Configure-Begin
	@cp Config.h.template Config.h.temp
	@$(call CONFIG_SETVAR,Config.h.temp,USE_COLLABORATION,$(HAVE_COLLABORATION))
	@$(call CONFIG_SETSTRINGVAR,Config.h.temp,LIDARVIEWER_CONFIGFILENAME,$(ETCINSTALLDIR)/LidarViewer.cfg)
	@if ! diff -qN Config.h.temp Config.h > /dev/null ; then cp Config.h.temp Config.h ; fi
	@rm Config.h.temp
	@touch $(DEPDIR)/Configure-LidarViewer

$(DEPDIR)/Configure-Install: $(DEPDIR)/Configure-LidarViewer
	@echo "---- $(PROJECT_FULLDISPLAYNAME) installation configuration ----"
	@echo "Installation directory : $(INSTALLDIR)"
	@echo "Executable directory   : $(EXECUTABLEINSTALLDIR)"
	@echo "Configuration directory: $(ETCINSTALLDIR)"
	@touch $(DEPDIR)/Configure-Install

$(DEPDIR)/Configure-End: $(DEPDIR)/Configure-Install
	@echo "---- End of $(PROJECT_FULLDISPLAYNAME) configuration options ----"
	@touch $(DEPDIR)/Configure-End

$(DEPDIR)/config: $(DEPDIR)/Configure-End
	@touch $(DEPDIR)/config

########################################################################
# Specify other actions to be performed on a `make clean'
########################################################################

.PHONY: extraclean
extraclean:

.PHONY: extrasqueakyclean
extrasqueakyclean:
	-rm -f $(ALL)
	-rm -r $(CONFIGS)

# Include basic makefile
include $(VRUI_MAKEDIR)/BasicMakefile

########################################################################
# Specify build rules for executables
########################################################################

CALCLASRANGE_SOURCES = CalcLasRange.cpp

$(CALCLASRANGE_SOURCES:%.cpp=$(OBJDIR)/%.o): | $(DEPDIR)/config

$(EXEDIR)/CalcLasRange: PACKAGES += MYCOMM
$(EXEDIR)/CalcLasRange: $(OBJDIR)/CalcLasRange.o
.PHONY: CalcLasRange
CalcLasRange: $(EXEDIR)/CalcLasRange

LIDARPREPROCESSOR_SOURCES = SplitPoints.cpp \
                            TempOctree.cpp \
                            PointAccumulator.cpp \
                            LidarProcessOctree.cpp \
                            LidarOctreeCreator.cpp \
                            ReadPlyFile.cpp \
                            LidarPreprocessor.cpp

$(LIDARPREPROCESSOR_SOURCES:%.cpp=$(OBJDIR)/%.o): | $(DEPDIR)/config

$(EXEDIR)/LidarPreprocessor: PACKAGES += MYIMAGES MYCOMM MYTHREADS
$(EXEDIR)/LidarPreprocessor: $(LIDARPREPROCESSOR_SOURCES:%.cpp=$(OBJDIR)/%.o)
.PHONY: LidarPreprocessor
LidarPreprocessor: $(EXEDIR)/LidarPreprocessor

LIDARSPOTREMOVER_SOURCES = LidarProcessOctree.cpp \
                           LidarSpotRemover.cpp

$(LIDARSPOTREMOVER_SOURCES:%.cpp=$(OBJDIR)/%.o): | $(DEPDIR)/config

$(EXEDIR)/LidarSpotRemover: $(LIDARSPOTREMOVER_SOURCES:%.cpp=$(OBJDIR)/%.o)
.PHONY: LidarSpotRemover
LidarSpotRemover: $(EXEDIR)/LidarSpotRemover

LIDARSUBTRACTOR_SOURCES = LidarProcessOctree.cpp \
                          SplitPoints.cpp \
                          TempOctree.cpp \
                          LidarOctreeCreator.cpp \
                          PointAccumulator.cpp \
                          SubtractorHelper.cpp \
                          LidarSubtractor.cpp

$(LIDARSUBTRACTOR_SOURCES:%.cpp=$(OBJDIR)/%.o): | $(DEPDIR)/config

$(EXEDIR)/LidarSubtractor: $(LIDARSUBTRACTOR_SOURCES:%.cpp=$(OBJDIR)/%.o)
.PHONY: LidarSubtractor
LidarSubtractor: $(EXEDIR)/LidarSubtractor

LIDARILLUMINATOR_SOURCES = LidarProcessOctree.cpp \
                           NormalCalculator.cpp \
                           LidarIlluminator.cpp

$(LIDARILLUMINATOR_SOURCES:%.cpp=$(OBJDIR)/%.o): | $(DEPDIR)/config

$(EXEDIR)/LidarIlluminator: $(LIDARILLUMINATOR_SOURCES:%.cpp=$(OBJDIR)/%.o)
.PHONY: LidarIlluminator
LidarIlluminator: $(EXEDIR)/LidarIlluminator

LIDARCOLORMAPPER_SOURCES = LidarProcessOctree.cpp \
                           LidarColorMapper.cpp

$(LIDARCOLORMAPPER_SOURCES:%.cpp=$(OBJDIR)/%.o): | $(DEPDIR)/config

$(EXEDIR)/LidarColorMapper: PACKAGES += MYIMAGES
$(EXEDIR)/LidarColorMapper: $(LIDARCOLORMAPPER_SOURCES:%.cpp=$(OBJDIR)/%.o)
.PHONY: LidarColorMapper
LidarColorMapper: $(EXEDIR)/LidarColorMapper

PAULBUNYAN_SOURCES = LidarProcessOctree.cpp \
                     PaulBunyan.cpp

$(PAULBUNYAN_SOURCES:%.cpp=$(OBJDIR)/%.o): | $(DEPDIR)/config

$(EXEDIR)/PaulBunyan: PACKAGES += MYIMAGES
$(EXEDIR)/PaulBunyan: $(PAULBUNYAN_SOURCES:%.cpp=$(OBJDIR)/%.o)
.PHONY: PaulBunyan
PaulBunyan: $(EXEDIR)/PaulBunyan

LIDAREXPORTER_SOURCES = LidarProcessOctree.cpp \
                        LidarExporter.cpp

$(LIDAREXPORTER_SOURCES:%.cpp=$(OBJDIR)/%.o): | $(DEPDIR)/config

$(EXEDIR)/LidarExporter: $(LIDAREXPORTER_SOURCES:%.cpp=$(OBJDIR)/%.o)
.PHONY: LidarExporter
LidarExporter: $(EXEDIR)/LidarExporter

LIDARGRIDDER_SOURCES = LidarProcessOctree.cpp \
                       LidarGridder.cpp

$(LIDARGRIDDER_SOURCES:%.cpp=$(OBJDIR)/%.o): | $(DEPDIR)/config

$(EXEDIR)/LidarGridder: PACKAGES += MYVRUI MYGLGEOMETRY MYGLSUPPORT MYGLWRAPPERS GL
$(EXEDIR)/LidarGridder: $(LIDARGRIDDER_SOURCES:%.cpp=$(OBJDIR)/%.o)
.PHONY: LidarGridder
LidarGridder: $(EXEDIR)/LidarGridder

LIDARVIEWER_SOURCES = LidarOctree.cpp \
                      PointBasedLightingShader.cpp \
                      ProjectorTool.cpp \
                      PointSelectorTool.cpp \
                      Primitive.cpp \
                      PointPrimitive.cpp \
                      SpherePrimitive.cpp \
                      LinePrimitive.cpp \
                      CylinderPrimitive.cpp \
                      PlanePrimitive.cpp \
                      BruntonPrimitive.cpp \
                      PrimitiveDraggerTool.cpp \
                      SceneGraph.cpp \
                      LidarProcessOctree.cpp \
                      LoadPointSet.cpp \
                      LidarViewer.cpp

$(LIDARVIEWER_SOURCES:%.cpp=$(OBJDIR)/%.o): | $(DEPDIR)/config

$(EXEDIR)/LidarViewer: PACKAGES += MYVRUI MYSCENEGRAPH MYGLMOTIF MYGLGEOMETRY MYGLSUPPORT MYGLWRAPPERS MYCLUSTER GL
ifneq ($(HAVE_COLLABORATION),0)
  $(EXEDIR)/LidarViewer: PACKAGES += MYCOLLABORATION2CLIENT
endif
# $(EXEDIR)/LidarViewer: CFLAGS += -DVISUALIZE_WATER
$(EXEDIR)/LidarViewer: $(LIDARVIEWER_SOURCES:%.cpp=$(OBJDIR)/%.o)
.PHONY: LidarViewer
LidarViewer: $(EXEDIR)/LidarViewer

$(OBJDIR)/PointSetSimilarity.o: | $(DEPDIR)/config

$(EXEDIR)/PointSetSimilarity: $(OBJDIR)/PointSetSimilarity.o
.PHONY: PointSetSimilarity
PointSetSimilarity: $(EXEDIR)/PointSetSimilarity

$(OBJDIR)/PrintPrimitiveFile.o: | $(DEPDIR)/config

$(EXEDIR)/PrintPrimitiveFile: $(OBJDIR)/PrintPrimitiveFile.o
.PHONY: PrintPrimitiveFile
PrintPrimitiveFile: $(EXEDIR)/PrintPrimitiveFile

install:
	@echo Installing $(PROJECT_FULLDISPLAYNAME) in $(INSTALLDIR)...
	@install -d $(INSTALLDIR)
	@echo Installing executables in $(EXECUTABLEINSTALLDIR)
	@install -d $(EXECUTABLEINSTALLDIR)
	@install $(EXECUTABLES) $(EXECUTABLEINSTALLDIR)
	@echo Installing configuration files in $(ETCINSTALLDIR)
	@install -d $(ETCINSTALLDIR)
	@install -m u=rw,go=r $(PROJECT_ETCDIR)/LidarViewer.cfg $(ETCINSTALLDIR)
