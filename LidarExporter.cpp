/***********************************************************************
LidarExporter - Utility to export points from LiDAR files to ASCII files
in xyzrgb format.
Copyright (c) 2010-2023 Oliver Kreylos

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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <Misc/File.h>
#include <IO/SeekableFile.h>
#include <IO/OpenFile.h>

#include "LidarTypes.h"
#include "LidarProcessOctree.h"

class PointSaver
	{
	/* Elements: */
	private:
	LidarProcessOctree::OffsetVector offset;
	Misc::File resultFile;
	size_t numPoints;
	
	/* Constructors and destructors: */
	public:
	PointSaver(const LidarProcessOctree::OffsetVector& sOffset,const char* resultFileName)
		:offset(sOffset),
		 resultFile(resultFileName,"wb"),
		 numPoints(0)
		{
		}
	
	/* Methods: */
	void operator()(const LidarPoint& point)
		{
		double pos[3];
		for(int i=0;i<3;++i)
			pos[i]=double(point[i])+double(offset[i]);
		fprintf(resultFile.getFilePtr(),"%.12g %.12g %.12g %u %u %u\n",pos[0],pos[1],pos[2],point.value[0],point.value[1],point.value[2]);
		++numPoints;
		}
	size_t getNumPoints(void) const
		{
		return numPoints;
		}
	};

class BinaryPointSaver
	{
	/* Elements: */
	private:
	IO::SeekableFilePtr resultFile;
	size_t numPoints;
	
	/* Constructors and destructors: */
	public:
	BinaryPointSaver(const LidarProcessOctree::OffsetVector& offset,const char* resultFileName)
		:resultFile(IO::openSeekableFile(resultFileName,IO::File::WriteOnly)),
		 numPoints(0)
		{
		resultFile->setEndianness(Misc::LittleEndian);
		
		/* Write the point offset vector: */
		resultFile->write(offset.getComponents(),3);
		
		/* Write the initial number of points: */
		resultFile->write<unsigned int>(0);
		}
	~BinaryPointSaver(void)
		{
		/* Write the final number of points: */
		resultFile->setWritePosAbs(3*sizeof(double));
		resultFile->write<unsigned int>(numPoints);
		}
	
	/* Methods: */
	void operator()(const LidarPoint& point)
		{
		/* Write the point: */
		resultFile->write(point.getComponents(),3);
		resultFile->write(point.value.getRgba(),4);
		
		/* Update the file header: */
		++numPoints;
		}
	size_t getNumPoints(void) const
		{
		return numPoints;
		}
	};

class LasPointSaver
	{
	/* Elements: */
	private:
	IO::SeekableFilePtr lasFile;
	LidarProcessOctree::OffsetVector lpoOffset;
	double scale[3],offset[3];
	double min[3],max[3];
	size_t numPoints;
	
	/* Constructors and destructors: */
	public:
	LasPointSaver(const char* lasFileName,const LidarProcessOctree::OffsetVector& sLpoOffset,const double sScale[3],const double sOffset[3])
		:lasFile(IO::openSeekableFile(lasFileName,IO::File::WriteOnly)),
		 lpoOffset(sLpoOffset),
		 numPoints(0)
		{
		for(int i=0;i<3;++i)
			{
			scale[i]=sScale[i];
			offset[i]=sOffset[i];
			min[i]=Math::Constants<double>::max;
			max[i]=Math::Constants<double>::min;
			}
		
		/* Create the initial LAS file header: */
		char signature[5]="LASF";
		lasFile->write(signature,4); // LAS signature
		lasFile->write<unsigned short>(0); // File source ID
		lasFile->write<unsigned short>(0); // Reserved field
		lasFile->write<unsigned int>(0); // Project ID
		lasFile->write<unsigned short>(0); // Project ID
		lasFile->write<unsigned short>(0); // Project ID
		char dummy[32];
		memset(dummy,0,sizeof(dummy));
		lasFile->write(dummy,8); // Project ID
		lasFile->write<unsigned char>(1); // File version number
		lasFile->write<unsigned char>(2); // File version number
		lasFile->write(dummy,32); // System identifier
		lasFile->write(dummy,32); // Generating software
		lasFile->write<unsigned short>(1); // File creation day of year
		lasFile->write<unsigned short>(2011); // File creation year
		lasFile->write<unsigned short>(227); // LAS header size
		lasFile->write<unsigned int>(227); // Point data offset
		lasFile->write<unsigned int>(0); // Number of variable-length records
		lasFile->write<unsigned char>(2); // Point data format
		lasFile->write<unsigned short>(26); // Point data record length
		lasFile->write<unsigned int>(0); // Number of point records
		lasFile->write<unsigned int>(0); // Number of point records by return
		lasFile->write<unsigned int>(0); // Number of point records by return
		lasFile->write<unsigned int>(0); // Number of point records by return
		lasFile->write<unsigned int>(0); // Number of point records by return
		lasFile->write<unsigned int>(0); // Number of point records by return
		lasFile->write(scale,3); // Quantization scale factor
		for(int i=0;i<3;++i) // Quantization offset vector
			lasFile->write<double>(offset[i]+lpoOffset[i]);
		for(int i=0;i<3;++i) // Point position bounding box
			{
			lasFile->write<double>(max[i]);
			lasFile->write<double>(min[i]);
			}
		}
	~LasPointSaver(void)
		{
		/* Write the final LAS header: */
		lasFile->setWritePosAbs(107);
		lasFile->write<unsigned int>(numPoints);
		lasFile->write<unsigned int>(numPoints);
		lasFile->setWritePosAbs(179);
		for(int i=0;i<3;++i)
			{
			lasFile->write<double>(max[i]+lpoOffset[i]);
			lasFile->write<double>(min[i]+lpoOffset[i]);
			}
		}
	
	/* Methods: */
	void operator()(const LidarPoint& point)
		{
		/* Quantize the point position: */
		int p[3];
		for(int i=0;i<3;++i)
			p[i]=int(Math::floor((double(point[i])-offset[i])/scale[i]+0.5));
		
		/* Calculate point intensity from RGB color: */
		unsigned short intensity=((unsigned short)(point.value[0])+(unsigned short)(point.value[1])+(unsigned short)(point.value[2])+1)/3;
		
		/* Write the point record: */
		lasFile->write(p,3); // Quantized point position
		lasFile->write<unsigned short>(intensity); // Point intensity
		lasFile->write<char>(0); // Return data
		lasFile->write<char>(0); // Point classification
		lasFile->write<unsigned char>(0); // Laser angle
		lasFile->write<unsigned char>(0); // User data
		lasFile->write<unsigned short>(0); // Source
		lasFile->write<unsigned short>(point.value.getRgba(),3);
		
		/* Update LAS header: */
		for(int i=0;i<3;++i)
			{
			if(min[i]>point[i])
				min[i]=point[i];
			if(max[i]<point[i])
				max[i]=point[i];
			}
		++numPoints;
		}
	size_t getNumPoints(void) const
		{
		return numPoints;
		}
	};

int main(int argc,char* argv[])
	{
	/* Process command line: */
	const char* lidarFile=0;
	const char* colorsFileName=0;
	int outputFileType=0;
	double lasScale[3]={0.001,0.001,0.001};
	const char* outputFile=0;
	int cacheSize=512;
	bool haveBox=false;
	double box[6];
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"readColors")==0)
				{
				++i;
				colorsFileName=argv[i];
				}
			else if(strcasecmp(argv[i]+1,"cache")==0)
				{
				++i;
				cacheSize=atoi(argv[i]);
				}
			else if(strcasecmp(argv[i]+1,"box")==0)
				{
				haveBox=true;
				for(int j=0;j<6;++j)
					{
					++i;
					box[j]=atof(argv[i]);
					}
				}
			else if(strcasecmp(argv[i]+1,"bin")==0)
				outputFileType=1;
			else if(strcasecmp(argv[i]+1,"las")==0)
				outputFileType=2;
			else if(strcasecmp(argv[i]+1,"lasScale")==0)
				{
				if(i+3<argc)
					{
					for(int j=0;j<3;++j)
						{
						++i;
						lasScale[j]=atof(argv[i]);
						}
					}
				else
					{
					std::cerr<<"Ignoring dangling -lasScale option"<<std::endl;
					i+=3;
					}
				}
			else
				std::cerr<<"Ignoring command line option "<<argv[i]<<std::endl;
			}
		else if(lidarFile==0)
			lidarFile=argv[i];
		else if(outputFile==0)
			outputFile=argv[i];
		else
			std::cerr<<"Ignoring command line argument "<<argv[i]<<std::endl;
		}
	if(lidarFile==0||outputFile==0)
		{
		std::cerr<<"Usage: "<<argv[0]<<" [-cache <cache size>] [-box <box spec>] <LiDAR file name> [-bin] [-las] [-lasScale <x scale> <y scale> <z scale>] <output file name>"<<std::endl;
		std::cerr<<"  -readColors <colors file name> requests to read the additional point color file of the given name"<<std::endl;
		std::cerr<<"  -cache <cache size> sets the size of the LiDAR memory cache in MB (default: 512)"<<std::endl;
		std::cerr<<"  -box <box spec> specifies a box in source coordinates from which to export points (default: export all points)"<<std::endl;
		std::cerr<<"     box specification: <min_x> <min_y> <min_z> <max_x> <max_y> <max_z>"<<std::endl;
		std::cerr<<"  -bin requests to write exported points into a binary file (default: write into ASCII file)"<<std::endl;
		std::cerr<<"  -las requests to write exported points into a LAS-like file (default: write into ASCII file)"<<std::endl;
		std::cerr<<"  -lasScale <x scale> <y scale> <z scale> defines the quantization scaling factors for LAS files"<<std::endl;
		return 1;
		}
	
	/* Open the input and output files: */
	LidarProcessOctree lpo(lidarFile,size_t(cacheSize)*1024*1024,colorsFileName);
	Box lbox;
	if(haveBox)
		{
		/* Transform the box to LiDAR coordinates: */
		for(int i=0;i<3;++i)
			{
			lbox.min[i]=Box::Scalar(box[i]-lpo.getOffset()[i]);
			lbox.max[i]=Box::Scalar(box[3+i]-lpo.getOffset()[i]);
			}
		}
	
	if(outputFileType==2)
		{
		/* Use octree file's center point as quantization offset: */
		double lasOffset[3];
		for(int i=0;i<3;++i)
			lasOffset[i]=lpo.getDomain().getCenter(i);
		LasPointSaver ps(outputFile,lpo.getOffset(),lasScale,lasOffset);
		
		/* Process the LiDAR file: */
		if(haveBox)
			{
			/* Exports points from inside the box: */
			lpo.processPointsInBox(lbox,ps);
			}
		else
			{
			/* Export all points: */
			lpo.processPoints(ps);
			}
		
		/* Print statistics: */
		std::cout<<ps.getNumPoints()<<" points saved"<<std::endl;
		}
	else if(outputFileType==1)
		{
		BinaryPointSaver ps(lpo.getOffset(),outputFile);
		
		/* Process the LiDAR file: */
		if(haveBox)
			{
			/* Exports points from inside the box: */
			lpo.processPointsInBox(lbox,ps);
			}
		else
			{
			/* Export all points: */
			lpo.processPoints(ps);
			}
		
		/* Print statistics: */
		std::cout<<ps.getNumPoints()<<" points saved"<<std::endl;
		}
	else
		{
		PointSaver ps(lpo.getOffset(),outputFile);
		
		/* Process the LiDAR file: */
		if(haveBox)
			{
			/* Exports points from inside the box: */
			lpo.processPointsInBox(lbox,ps);
			}
		else
			{
			/* Export all points: */
			lpo.processPoints(ps);
			}
		
		/* Print statistics: */
		std::cout<<ps.getNumPoints()<<" points saved"<<std::endl;
		}
	
	return 0;
	}
