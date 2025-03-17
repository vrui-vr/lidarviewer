/***********************************************************************
New version of LiDAR data preprocessor.
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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <Misc/StdError.h>
#include <Misc/Endianness.h>
#include <Misc/FileNameExtensions.h>
#include <Misc/Timer.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <IO/File.h>
#include <IO/SeekableFile.h>
#include <IO/OpenFile.h>
#include <IO/ReadAheadFilter.h>
#include <IO/ValueSource.h>
#include <IO/OpenFile.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/GeometryValueCoders.h>
#include <Images/BaseImage.h>
#include <Images/ImageReaderBIL.h>
#include <Images/GeoTIFFMetadata.h>
#include <Images/TIFFReader.h>

#include "Config.h"
#include "LidarTypes.h"
#include "PointAccumulator.h"
#include "ReadPlyFile.h"
#include "LidarProcessOctree.h"
#include "LidarOctreeCreator.h"

class TIFFDEMLoader // Helper class to convert DEMs in TIFF format into a point cloud
	{
	/* Elements: */
	private:
	Images::TIFFReader tiffReader; // TIFF image file reader object
	Images::GeoTIFFMetadata mapData; // Map transformation defined in the TIFF image file
	PointAccumulator& pa; // Accumulator for 3D points
	PointAccumulator::Color color; // Color to assign to all points
	
	/* Private methods: */
	static void streamingCallback(uint32_t x,uint32_t y,uint32_t width,uint16_t channel,const uint8_t* pixels,void* userData) // Callback receiving pixel or channel data from the TIFF file
		{
		TIFFDEMLoader* thisPtr=static_cast<TIFFDEMLoader*>(userData);
		Images::GeoTIFFMetadata& md=thisPtr->mapData;
		
		/* Convert all elevation postings into 3D points and add them to the point accumulator: */
		const float* pPtr=reinterpret_cast<const float*>(pixels);
		PointAccumulator::Point p;
		p[0]=double(x)*md.dim[0]+md.map[0];
		p[1]=double(y)*md.dim[1]+md.map[1];
		while(width>0)
			{
			/* Convert and add the current pixel: */
			p[2]=double(*pPtr);
			if(p[2]!=md.noData)
				thisPtr->pa.addPoint(p,thisPtr->color);
			
			/* Go to the next pixel: */
			p[0]+=md.dim[0];
			--width;
			++pPtr;
			}
		}
	
	/* Constructors and destructors: */
	public:
	TIFFDEMLoader(PointAccumulator& sPa,const char* fileName,unsigned int imageIndex)
		:tiffReader(*IO::openFile(fileName),imageIndex),
		 pa(sPa),color(1.0,1.0,1.0)
		{
		/* Check if the TIFF image has a compatible format: */
		if(tiffReader.getNumSamples()!=1||tiffReader.getNumBits()!=32||!tiffReader.hasFloatSamples())
			throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Image %u in TIFF file %s does not define a DEM",imageIndex,fileName);
		
		std::cout<<"Reading "<<tiffReader.getWidth()<<" x "<<tiffReader.getHeight()<<" DEM pixels"<<std::endl;
		
		/* Read the TIFF image's map transformation, if it defines one: */
		tiffReader.readMetadata(mapData);
		
		/* OK, the input file doesn't have GeoTIFF tags: */
		mapData.haveMap=true;
		mapData.map[0]=8127993.99279;
		mapData.map[1]=-244781.57519;
		mapData.haveDim=true;
		mapData.dim[0]=1.0;
		mapData.dim[1]=-1.0;
		mapData.haveNoData=true;
		mapData.noData=-32767.0;
		
		/* Flip the map data: */
		mapData.map[1]=mapData.map[1]+mapData.dim[1]*double(tiffReader.getHeight()-1);
		mapData.dim[1]=-mapData.dim[1];
		
		if(mapData.haveMap)
			std::cout<<"Map position of upper-left DEM pixel: "<<mapData.map[0]<<", "<<mapData.map[1]<<std::endl;
		if(mapData.haveDim)
			std::cout<<"Map dimensions of DEM pixels: "<<mapData.dim[0]<<" x "<<mapData.dim[1]<<std::endl;
		}
	
	/* Methods: */
	void collectPoints(void) // Collects all 3D points defined by the DEM into the point accumulator
		{
		/* Stream the image's pixels into the streaming callback, which in turn adds them to the point accumulator: */
		tiffReader.streamImage(&TIFFDEMLoader::streamingCallback,this);
		}
	};

inline bool startsNumber(int c)
	{
	return (c>='0'&&c<='9')||c=='+'||c=='-'||c=='.';
	}

std::string readNumber(IO::ValueSource& header)
	{
	/* Read the number as a string: */
	std::string result=header.readString();
	
	/* Skip a potential unit name: */
	if(header.peekc()=='<')
		{
		while(header.getChar()!='>')
			;
		header.skipWs();
		}
	
	return result;
	}

std::string readList(IO::ValueSource& header,bool root)
	{
	std::string result;
	
	/* Read the opening parenthesis: */
	result.push_back(header.readChar());
	
	if(root)
		header.setWhitespace('\n',true);
	
	/* Read list elements until the closing parenthesis: */
	while(header.peekc()!=')')
		{
		/* Check the type of the next element: */
		if(header.peekc()=='(')
			result.append(readList(header,false));
		else if(header.peekc()=='"')
			{
			result.push_back('"');
			result.append(header.readString());
			result.push_back('"');
			}
		else if(startsNumber(header.peekc()))
			result.append(readNumber(header));
		else
			result.append(header.readString());
		
		/* Check for the item separator: */
		if(header.peekc()!=')')
			{
			if(!header.isLiteral(','))
				throw std::runtime_error("Missing comma separator in list value");
			result.push_back(',');
			}
		}
	
	if(root)
		header.setWhitespace('\n',false);
	
	/* Read the closing parenthesis: */
	result.push_back(header.readChar());
	
	return result;
	}

void loadXYZBILImage(PointAccumulator& pa,const char* fileName)
	{
	/* Open the image file: */
	IO::SeekableFilePtr file=IO::openSeekableFile(fileName);
	
	/* Read the embedded file header as plain text: */
	size_t recordSize=0;
	size_t imageOffset=0;
	bool inImageObject=false;
	Images::ImageReaderBIL::FileLayout fileLayout;
	fileLayout.size=Images::Size(0,0);
	fileLayout.numBands=3;
	fileLayout.numBits=32;
	fileLayout.pixelSigned=true;
	fileLayout.byteOrder=Misc::BigEndian;
	fileLayout.bandLayout=Images::ImageReaderBIL::FileLayout::BSQ;
	fileLayout.skipBytes=0;
	fileLayout.bandRowBytes=0;
	fileLayout.totalRowBytes=0;
	fileLayout.bandGapBytes=0;
	{
	IO::ValueSource header(file);
	header.setPunctuation("=,()<>\n");
	header.setQuote('"',true);
	header.skipWs();
	
	/* Check if the file is an ODL3 data product: */
	if(!header.isLiteral("ODL_VERSION_ID")||!header.isLiteral('=')||!header.isLiteral("ODL3")||!header.isLiteral('\n'))
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"File %s is not an ODL3 data product",fileName);
	
	/* Read the header line-by-line: */
	while(true)
		{
		/* Check for comment lines: */
		if(header.peekc()!='\n'&&(header.peekc()!='/'||header.getChar()!='/'||header.peekc()!='*'))
			{
			/* Read the line tag: */
			std::string tag=header.readString();
			if(tag=="END")
				{
				/* Done with the header: */
				break;
				}
			
			/* Check for the equal sign: */
			if(!header.isLiteral('='))
				throw Misc::makeStdErr(__PRETTY_FUNCTION__,"File %s has a malformed ODL3 header",fileName);
			
			/* Read the line value: */
			std::string value;
			if(header.peekc()=='(')
				value=readList(header,true);
			else if(header.peekc()=='"')
				{
				header.setQuotedString('\n',true);
				value.push_back('"');
				value.append(header.readString());
				value.push_back('"');
				header.setQuotedString('\n',false);
				}
			else if(startsNumber(header.peekc()))
				value=readNumber(header);
			else
				value=header.readString();
			
			/* Parse relevant tags: */
			if(inImageObject)
				{
				if(tag=="INTERCHANGE_FORMAT")
					{
					if(value!="BINARY")
						throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Unsupported interchange format %s in file %s",value.c_str(),fileName);
					}
				else if(tag=="LINES")
					fileLayout.size[1]=Misc::ValueCoder<unsigned int>::decode(value.data(),value.data()+value.length());
				else if(tag=="LINE_SAMPLES")
					fileLayout.size[0]=Misc::ValueCoder<unsigned int>::decode(value.data(),value.data()+value.length());
				else if(tag=="SAMPLE_TYPE")
					{
					if(value!="IEEE_REAL")
						throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Unsupported sample type %s in file %s",value.c_str(),fileName);
					}
				else if(tag=="SAMPLE_BITS")
					{
					fileLayout.numBits=Misc::ValueCoder<unsigned int>::decode(value.data(),value.data()+value.length());
					if(fileLayout.numBits!=32)
						throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Unsupported sample size %u in file %s",(unsigned int)(fileLayout.numBits),fileName);
					}
				else if(tag=="BANDS")
					{
					fileLayout.numBands=Misc::ValueCoder<unsigned int>::decode(value.data(),value.data()+value.length());
					if(fileLayout.numBands!=3)
						throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Unsupported number of bands %u in file %s",(unsigned int)(fileLayout.numBands),fileName);
					}
				else if(tag=="BAND_STORAGE_TYPE")
					{
					if(value!="BAND_SEQUENTIAL")
						throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Unsupported band storage type %s in file %s",value.c_str(),fileName);
					}
				else if(tag=="END_OBJECT")
					inImageObject=value!="IMAGE";
				}
			else
				{
				if(tag=="RECORD_TYPE")
					{
					if(value!="FIXED_LENGTH")
						throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Unsupported record type %s in file %s",value.c_str(),fileName);
					}
				else if(tag=="RECORD_BYTES")
					recordSize=Misc::ValueCoder<unsigned int>::decode(value.data(),value.data()+value.length());
				else if(tag=="^IMAGE")
					imageOffset=Misc::ValueCoder<unsigned int>::decode(value.data(),value.data()+value.length())-1;
				else if(tag=="OBJECT")
					inImageObject=value=="IMAGE";
				}
			}
		
		/* Skip the rest of the line: */
		header.skipLine();
		header.skipWs();
		}
	}
	
	/* Seek the file to the beginning of the image data: */
	file->setReadPosAbs(recordSize*imageOffset);
	
	/* Read the image: */
	Images::BaseImage xyzImage=Images::ImageReaderBIL(fileLayout,*file).readImage();
	
	/* Check that the image has three floating-point bands: */
	if(xyzImage.getNumChannels()!=3||xyzImage.getChannelSize()!=sizeof(float)||xyzImage.getScalarType()!=GL_FLOAT)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"File %s is not an XYZ image file",fileName);
	
	/* Add all non-zero image pixels to the point accumulator: */
	const float* pPtr=static_cast<const float*>(xyzImage.getPixels());
	for(size_t numPixels=xyzImage.getHeight()*xyzImage.getWidth();numPixels>0;--numPixels,pPtr+=3)
		if(pPtr[0]!=0.0f||pPtr[1]!=0.0f||pPtr[2]!=0.0f)
			pa.addPoint(PointAccumulator::Point(-pPtr[0],pPtr[1],-pPtr[2]),PointAccumulator::Color(1,1,1)); // X and Z coordinates must be negated!
	}

void loadPointFileBin(PointAccumulator& pa,const char* fileName)
	{
	/* Open the binary input file: */
	IO::FilePtr file(new IO::ReadAheadFilter(IO::openFile(fileName)));
	file->setEndianness(Misc::LittleEndian);
	
	/* Read the number of points in the file: */
	size_t numPoints=file->read<unsigned int>();
	
	/* Read all points: */
	for(size_t i=0;i<numPoints;++i)
		{
		/* Read the point position and intensity from the input file: */
		float rp[4];
		file->read(rp,4);
		
		/* Store the point: */
		pa.addPoint(PointAccumulator::Point(rp),PointAccumulator::Color(rp[3],rp[3],rp[3]));
		}
	}

void loadPointFileBinRgb(PointAccumulator& pa,const char* fileName)
	{
	/* Open the binary input file: */
	IO::FilePtr file(new IO::ReadAheadFilter(IO::openFile(fileName)));
	file->setEndianness(Misc::LittleEndian);
	
	/* Read the number of points in the file: */
	size_t numPoints=file->read<unsigned int>();
	
	/* Read all points: */
	for(size_t i=0;i<numPoints;++i)
		{
		/* Read the point position and color from the input file: */
		float rp[3];
		file->read(rp,3);
		Color::Scalar rcol[4];
		file->read(rcol,4);
		
		/* Store the point: */
		pa.addPoint(PointAccumulator::Point(rp),PointAccumulator::Color(rcol));
		}
	}

void loadPointFileLas(PointAccumulator& pa,const char* fileName,unsigned int classMask)
	{
	/* Open the LAS input file: */
	IO::FilePtr file(new IO::ReadAheadFilter(IO::openFile(fileName)));
	file->setEndianness(Misc::LittleEndian);
	
	/* Read the LAS file header: */
	char signature[4];
	file->read(signature,4);
	if(memcmp(signature,"LASF",4)!=0)
		return;
	
	file->skip<unsigned short>(1); // Ignore file source ID
	file->skip<unsigned short>(1); // Ignore reserved field
	file->skip<unsigned int>(1); // Ignore project ID
	file->skip<unsigned short>(1); // Ignore project ID
	file->skip<unsigned short>(1); // Ignore project ID
	file->skip<char>(8); // Ignore project ID
	file->skip<char>(2); // Ignore file version number
	file->skip<char>(32); // Ignore system identifier
	file->skip<char>(32); // Ignore generating software
	file->skip<unsigned short>(1); // Ignore file creation day of year
	file->skip<unsigned short>(1); // Ignore file creation year
	file->skip<unsigned short>(1); // Ignore header size
	unsigned int pointDataOffset=file->read<unsigned int>();
	file->skip<unsigned int>(1); // Ignore number of variable-length records
	unsigned char pointDataFormat=file->read<unsigned char>();
	unsigned short pointDataRecordLength=file->read<unsigned short>();
	size_t numPointRecords=file->read<unsigned int>();
	unsigned int numPointsByReturn[5];
	file->read(numPointsByReturn,5);
	double scale[3];
	file->read(scale,3);
	PointAccumulator::Vector offset;
	file->read(offset.getComponents(),3);
	double min[3],max[3];
	for(int i=0;i<3;++i)
		{
		max[i]=file->read<double>();
		min[i]=file->read<double>();
		}
	
	/* Check the point record format for sanity: */
	static const unsigned short expectedPointDataRecordLengths[]=
		{
		20,28,26,34,57,63
		};
	if(pointDataFormat<6U&&pointDataRecordLength<expectedPointDataRecordLengths[pointDataFormat])
		{
		std::cout<<"Ignoring LAS file "<<fileName<<" with point data format "<<int(pointDataFormat)<<" due to wrong point record length ("<<pointDataRecordLength<<" instead of "<<expectedPointDataRecordLengths[pointDataFormat]<<")"<<std::endl;
		return;
		}
	else if(pointDataFormat>=6U)
		{
		std::cout<<"Ignoring LAS file "<<fileName<<" due to unknown point data format "<<int(pointDataFormat)<<std::endl;
		return;
		}
	
	/* Arrays of point data record features by point data format: */
	static const bool haveTimes[]=
		{
		false,true,false,true,true,true
		};
	static const bool haveRgb[]=
		{
		false,false,true,true,false,false
		};
	size_t skipToClass=sizeof(char);
	size_t skipToColor=2*sizeof(unsigned char)+sizeof(unsigned short);
	if(haveTimes[pointDataFormat])
		skipToColor+=sizeof(double);
	size_t skipToEnd=pointDataRecordLength-expectedPointDataRecordLengths[pointDataFormat];
	
	/* Skip to the beginning of the point records: */
	if(pointDataOffset<227)
		{
		std::cout<<"Ignoring LAS file "<<fileName<<" due to short file header ("<<pointDataOffset<<" bytes)"<<std::endl;
		return;
		}
	file->skip<unsigned char>(pointDataOffset-227);
	
	/* Apply the LAS offset to the point accumulator's point offset: */
	PointAccumulator::Vector originalPointOffset=pa.getPointOffset();
	pa.setPointOffset(originalPointOffset+offset);
	
	/* Read all points: */
	for(size_t i=0;i<numPointRecords;++i)
		{
		/* Read the point position: */
		int pos[3];
		file->read(pos,3);
		PointAccumulator::Point p;
		for(int j=0;j<3;++j)
			p[j]=double(pos[j])*scale[j];
		
		/* Read the point intensity: */
		float intensity=float(file->read<unsigned short>());
		
		/* Skip to the point classification: */
		file->skip<char>(skipToClass);
		
		/* Read the point classification: */
		unsigned int classBit=0x1U<<(file->read<unsigned char>()&0x1fU);
		
		/* Skip to the (optional) point color: */
		file->skip<char>(skipToColor);
		
		PointAccumulator::Color c;
		if(haveRgb[pointDataFormat])
			{
			/* Assign point color from stored RGB data: */
			unsigned short rgb[3];
			file->read(rgb,3);
			for(int j=0;j<3;++j)
				c[j]=float(rgb[j]);
			}
		else
			{
			/* Assign point color from intensity: */
			for(int j=0;j<3;++j)
				c[j]=float(intensity);
			}
		
		/* Skip any extra data at the end of the point record: */
		file->skip<char>(skipToEnd);
		
		if(classMask&classBit)
			{
			/* Store the point: */
			pa.addPoint(p,c);
			}
		}
	
	/* Reset the point accumulator's point offset: */
	pa.setPointOffset(originalPointOffset);
	}

void loadPointFileXyzi(PointAccumulator& pa,const char* fileName)
	{
	/* Open the ASCII input file: */
	IO::ValueSource reader(new IO::ReadAheadFilter(IO::openFile(fileName)));
	reader.setPunctuation('#',true);
	reader.setPunctuation('\n',true);
	reader.skipWs();
	
	/* Read all lines from the input file: */
	size_t lineNumber=1;
	while(!reader.eof())
		{
		/* Check for comment or empty lines: */
		if(reader.peekc()!='#'&&reader.peekc()!='\n')
			{
			try
				{
				/* Parse the point coordinates and intensity from the line: */
				PointAccumulator::Point p;
				for(int i=0;i<3;++i)
					p[i]=reader.readNumber();
				float intensity=float(reader.readNumber());
				PointAccumulator::Color c(intensity,intensity,intensity);
				
				/* Store the point: */
				pa.addPoint(p,c);
				}
			catch(const IO::ValueSource::NumberError& err)
				{
				std::cerr<<"loadPointFileXyzi: Point parsing error in line "<<lineNumber<<std::endl;
				}
			}
		
		/* Skip the rest of the line: */
		reader.skipLine();
		++lineNumber;
		reader.skipWs();
		}
	}

void loadPointFileXyzrgb(PointAccumulator& pa,const char* fileName)
	{
	/* Open the ASCII input file: */
	IO::ValueSource reader(new IO::ReadAheadFilter(IO::openFile(fileName)));
	reader.setPunctuation('#',true);
	reader.setPunctuation('\n',true);
	reader.skipWs();
	
	/* Read all lines from the input file: */
	size_t lineNumber=1;
	while(!reader.eof())
		{
		/* Check for comment or empty lines: */
		if(reader.peekc()!='#'&&reader.peekc()!='\n')
			{
			try
				{
				/* Parse the point coordinates and color from the line: */
				PointAccumulator::Point p;
				for(int i=0;i<3;++i)
					p[i]=reader.readNumber();
				PointAccumulator::Color c;
				for(int i=0;i<3;++i)
					c[i]=float(reader.readNumber());
				
				/* Store the point: */
				pa.addPoint(p,c);
				}
			catch(const IO::ValueSource::NumberError& err)
				{
				std::cerr<<"loadPointFileXyzrgb: Point parsing error in line "<<lineNumber<<std::endl;
				}
			}
		
		/* Skip the rest of the line: */
		reader.skipLine();
		++lineNumber;
		reader.skipWs();
		}
	}

void loadPointFileGenericASCII(PointAccumulator& pa,const char* fileName,int numHeaderLines,bool strictCsv,bool rgb,const int columnIndices[6])
	{
	/* Create the mapping from column indices to point components: */
	int maxColumnIndex=columnIndices[0];
	for(int i=1;i<6;++i)
		if(maxColumnIndex<columnIndices[i])
			maxColumnIndex=columnIndices[i];
	int* componentColumnIndices=new int[maxColumnIndex+1];
	for(int i=0;i<=maxColumnIndex;++i)
		componentColumnIndices[i]=-1;
	int numComponents=0;
	for(int i=0;i<6;++i)
		if(columnIndices[i]>=0)
			{
			componentColumnIndices[columnIndices[i]]=i;
			++numComponents;
			}
	
	/* Create color components if they are not given: */
	if(rgb)
		{
		if(numComponents<6)
			numComponents=6;
		}
	else
		{
		if(numComponents<4)
			numComponents=4;
		}
	double* componentValues=new double[numComponents];
	
	/* Initialize the color components: */
	for(int i=3;i<numComponents;++i)
		componentValues[i]=255.0;
	
	/* Open the ASCII input file: */
	IO::ValueSource reader(new IO::ReadAheadFilter(IO::openFile(fileName)));
	if(strictCsv)
		reader.setWhitespace("");
	reader.setPunctuation("#,\n");
	reader.skipWs();
	size_t lineNumber=1;
	
	/* Skip the header lines: */
	for(int i=0;i<numHeaderLines;++i)
		{
		reader.skipLine();
		reader.skipWs();
		++lineNumber;
		}
	
	/* Read all lines from the input file: */
	try
		{
		while(!reader.eof())
			{
			/* Check for comments or empty lines: */
			if(reader.peekc()!='#'&&reader.peekc()!='\n')
				{
				/* Read all columns: */
				for(int columnIndex=0;columnIndex<=maxColumnIndex;++columnIndex)
					{
					if(componentColumnIndices[columnIndex]>=0)
						{
						/* Read the next value: */
						componentValues[componentColumnIndices[columnIndex]]=reader.readNumber();
						}
					else
						reader.skipString();
					
					if(columnIndex<maxColumnIndex)
						{
						/* Check for separator: */
						if(reader.peekc()==',')
							reader.skipString();
						}
					}
				
				/* Store the point position: */
				PointAccumulator::Point p(componentValues);
				
				PointAccumulator::Color c;
				if(rgb)
					{
					/* Modify the read RGB color according to the color mask: */
					for(int i=0;i<3;++i)
						c[i]=componentValues[3+i];
					}
				else
					{
					/* Convert the read intensity to an RGB color according to the color mask: */
					for(int i=0;i<3;++i)
						c[i]=componentValues[3];
					}
				
				/* Store the point: */
				pa.addPoint(p,c);
				}
			
			/* Skip to the next line: */
			reader.skipLine();
			reader.skipWs();
			++lineNumber;
			}
		}
	catch(const std::runtime_error& err)
		{
		std::cerr<<"Caught exception "<<err.what()<<" in line "<<lineNumber<<" in input file "<<fileName<<std::endl;
		}
	
	/* Clean up: */
	delete[] componentColumnIndices;
	delete[] componentValues;
	}

void loadPointFileBlockedASCII(PointAccumulator& pa,const char* fileName,int numHeaderLines,bool rgb,const int columnIndices[6])
	{
	/* Create the mapping from column indices to point components: */
	int maxColumnIndex=columnIndices[0];
	for(int i=1;i<6;++i)
		if(maxColumnIndex<columnIndices[i])
			maxColumnIndex=columnIndices[i];
	int* componentColumnIndices=new int[maxColumnIndex+1];
	for(int i=0;i<=maxColumnIndex;++i)
		componentColumnIndices[i]=-1;
	int numComponents=0;
	for(int i=0;i<6;++i)
		if(columnIndices[i]>=0)
			{
			componentColumnIndices[columnIndices[i]]=i;
			++numComponents;
			}
	double* componentValues=new double[numComponents];
	
	/* Open the ASCII input file: */
	IO::ValueSource reader(new IO::ReadAheadFilter(IO::openFile(fileName)));
	reader.setPunctuation("#,\n");
	reader.skipWs();
	size_t lineNumber=1;
	
	/* Skip the header lines: */
	for(int i=0;i<numHeaderLines;++i)
		{
		reader.skipLine();
		reader.skipWs();
		++lineNumber;
		}
	
	/* Read all lines from the input file: */
	try
		{
		while(!reader.eof())
			{
			/* Read the number of points in the next block: */
			int numPoints=reader.readInteger();
			reader.skipLine();
			reader.skipWs();
			++lineNumber;
			
			/* Read all points in the block: */
			for(int blockIndex=0;blockIndex<numPoints;++blockIndex)
				{
				/* Read all columns: */
				for(int columnIndex=0;columnIndex<=maxColumnIndex;++columnIndex)
					{
					if(componentColumnIndices[columnIndex]>=0)
						{
						/* Read the next value: */
						componentValues[componentColumnIndices[columnIndex]]=reader.readNumber();
						}
					else
						reader.skipString();
					
					if(columnIndex<maxColumnIndex)
						{
						/* Check for separator: */
						if(reader.peekc()==',')
							reader.skipString();
						}
					}
				
				/* Store the point position: */
				PointAccumulator::Point p(componentValues);
				
				/* Store the point color: */
				PointAccumulator::Color c;
				if(rgb)
					{
					/* Modify the read RGB color according to the color mask: */
					for(int i=0;i<3;++i)
						c[i]=float(componentValues[3+i]);
					}
				else
					{
					/* Convert the read intensity to an RGB color according to the color mask: */
					for(int i=0;i<3;++i)
						c[i]=float(componentValues[3]);
					}
				
				/* Store the point: */
				pa.addPoint(p,c);
				
				/* Skip to the next line: */
				reader.skipLine();
				reader.skipWs();
				++lineNumber;
				}
			}
		}
	catch(const std::runtime_error& err)
		{
		std::cerr<<"Caught exception "<<err.what()<<" in line "<<lineNumber<<" in input file "<<fileName<<std::endl;
		}
	
	/* Clean up: */
	delete[] componentColumnIndices;
	delete[] componentValues;
	}

/**********************************
Helper structure to load IDL files:
**********************************/

struct IDLFileRecord // Structure describing a record in an IDL file
	{
	/* Elements: */
	public:
	unsigned int galID[2]; // Galaxy ID (64-bit integer)
	unsigned int haloID[2]; // Halo ID (64-bit integer)
	unsigned int recordType; // 0=central, 1=sat, 2=orphan
	float position[3];
	float velocity[3];
	float spin[3];
	float ra;
	float dec;
	float z_obs;
	float z;
	float centralMVir;
	float mVir;
	float rVir;
	float vVir;
	float vMax;
	float velDisp;
	float stellarMass;
	float bulgeMass;
	float coldGas;
	float hotGas;
	float ejectedMass;
	float blackHoleMass;
	float sfr;
	float cooling;
	float heating;
	float appMagSdss[5];
	float appMagSdssBulge[5];
	float absMagSdss[5];
	float appMagDeep[4];
	float appMagDeepBulge[4];
	float absMagBvrik[5];
	float absMagBvrikBulge[5];
	float absMagBvrikNoDust[5];
	};

namespace Misc {

template <>
class EndiannessSwapper<IDLFileRecord>
	{
	/* Methods: */
	public:
	static void swap(IDLFileRecord& value)
		{
		swapEndianness(value.galID,2);
		swapEndianness(value.haloID,2);
		swapEndianness(value.recordType);
		swapEndianness(value.position,3);
		swapEndianness(value.velocity,3);
		swapEndianness(value.spin,3);
		swapEndianness(value.ra);
		swapEndianness(value.dec);
		swapEndianness(value.z_obs);
		swapEndianness(value.z);
		swapEndianness(value.centralMVir);
		swapEndianness(value.mVir);
		swapEndianness(value.rVir);
		swapEndianness(value.vVir);
		swapEndianness(value.vMax);
		swapEndianness(value.velDisp);
		swapEndianness(value.stellarMass);
		swapEndianness(value.bulgeMass);
		swapEndianness(value.coldGas);
		swapEndianness(value.hotGas);
		swapEndianness(value.ejectedMass);
		swapEndianness(value.blackHoleMass);
		swapEndianness(value.sfr);
		swapEndianness(value.cooling);
		swapEndianness(value.heating);
		swapEndianness(value.appMagSdss,5);
		swapEndianness(value.appMagSdssBulge,5);
		swapEndianness(value.absMagSdss,5);
		swapEndianness(value.appMagDeep,4);
		swapEndianness(value.appMagDeepBulge,4);
		swapEndianness(value.absMagBvrik,5);
		swapEndianness(value.absMagBvrikBulge,5);
		swapEndianness(value.absMagBvrikNoDust,5);
		}
	};

}

double angdiadistscaled(double z)
	{
	double h0=71.0;
	double oM=0.3;
	double oL=0.7;
	double oR=1.0-(oM+oL);
	
	double sum1=0.0;
	double dz=z/100.0;
	double id=1.0;
	for(int i=0;i<100;++i,id+=dz)
		{
		double ez=Math::sqrt((oM*id+oR)*id*id+oL);
		sum1+=dz/ez;
		}
	double dh=3.0e5/h0;
	double dc=dh*sum1;
	
	if(oR==0.0)
		return dh*sum1;
	else
		{
		double sqrtOR=Math::sqrt(Math::abs(oR));
		return dh*(1.0/sqrtOR)*sinh(sqrtOR*(sum1));
		}
	}

void loadPointFileIdl(PointAccumulator& pa,const char* fileName)
	{
	/* Open the IDL input file: */
	IO::FilePtr file(new IO::ReadAheadFilter(IO::openFile(fileName)));
	file->setEndianness(Misc::LittleEndian);
	
	/* Read the IDL file header: */
	size_t numRecords=file->read<unsigned int>();
	
	/* Read all records: */
	float massMin=Math::Constants<float>::max;
	float massMax=Math::Constants<float>::min;
	for(size_t i=0;i<numRecords;++i)
		{
		/* Read the next record: */
		IDLFileRecord record;
		file->read(record);
		
		/* Create a point: */
		PointAccumulator::Point p;
		#if 0
		for(int i=0;i<3;++i)
			p[i]=record.position[i];
		#else
		/* New formula using redshift to calculate galaxy position in Cartesian coordinates: */
		double z=3200.0*double(record.z);
		// double z=angdiadistscaled(record.z);
		p[0]=double(record.dec)*z;
		p[1]=double(record.ra)*z;
		p[2]=z;
		#endif
		
		if(massMin>record.mVir)
			massMin=record.mVir;
		if(massMax<record.mVir)
			massMax=record.mVir;
		float rgbFactor=(record.mVir/32565.4f)*0.5f+0.5f;
		
		/* Calculate false color from absolute SDSS magnitudes: */
		PointAccumulator::Color c;
		c[0]=(record.absMagSdss[2]-record.absMagSdss[3]+1.13f);
		c[1]=((-record.absMagSdss[2]-14.62f)*0.3);
		c[2]=(record.absMagSdss[1]-record.absMagSdss[2]+0.73f);
		
		// if(record.recordType==0)
			{
			/* Store the point: */
			pa.addPoint(p,c);
			}
		}
	
	std::cout<<"mVir range: "<<massMin<<" - "<<massMax<<std::endl;
	}

/**************************************
Helper structures to load octree files:
**************************************/

struct OldLidarOctreeFileHeader // Header structure for old LiDAR octree files
	{
	/* Elements: */
	public:
	Point center; // Center of the cube containing all LiDAR points
	Scalar radius; // Radius (half side length) of cube containing all LiDAR points
	unsigned int maxNumPointsPerNode; // Maximum number of points stored in each octree node
	
	/* Constructors and destructors: */
	OldLidarOctreeFileHeader(IO::File& file) // Reads the header from file
		{
		file.read(center.getComponents(),3);
		file.read(radius);
		file.read(maxNumPointsPerNode);
		}
	
	/* Methods: */
	static size_t getSize(void) // Returns the file size of the header
		{
		return sizeof(Point)+sizeof(Scalar)+sizeof(unsigned int);
		}
	};

struct OldLidarOctreeFileNode // Structure for a node in a LiDAR octree file
	{
	/* Elements: */
	public:
	IO::SeekableFile::Offset childrenOffset; // Offset of the node's children in the octree file (or 0 if leaf node)
	Scalar detailSize; // Detail size of this node, for proper LOD computation
	unsigned int numPoints; // Number of points in the node
	IO::SeekableFile::Offset pointsOffset; // Offset of the node's points in the points file
	
	/* Constructors and destructors: */
	OldLidarOctreeFileNode(IO::SeekableFile& file) // Reads the octree node from file
		{
		file.read(childrenOffset);
		file.read(detailSize);
		file.read(pointsOffset);
		file.read(numPoints);
		}
	
	/* Methods: */
	static size_t getSize(void) // Returns the file size of an octree node
		{
		return sizeof(IO::SeekableFile::Offset)+sizeof(Scalar)+sizeof(IO::SeekableFile::Offset)+sizeof(unsigned int);
		}
	};

void readOctreeFileSubtree(PointAccumulator& pa,IO::SeekableFile& octFile,IO::SeekableFile& obinFile)
	{
	/* Read the node's header from the octree file: */
	OldLidarOctreeFileNode ofn(octFile);
	
	if(ofn.childrenOffset!=0)
		{
		/* Recurse into the node's children: */
		IO::SeekableFile::Offset childOffset=ofn.childrenOffset;
		for(int childIndex=0;childIndex<8;++childIndex,childOffset+=OldLidarOctreeFileNode::getSize())
			{
			octFile.setReadPosAbs(childOffset);
			readOctreeFileSubtree(pa,octFile,obinFile);
			}
		}
	else if(ofn.numPoints>0)
		{
		/* Read the node's points from the octree point file: */
		obinFile.setReadPosAbs(ofn.pointsOffset);
		for(unsigned int i=0;i<ofn.numPoints;++i)
			{
			LidarPoint p;
			obinFile.read(p.getComponents(),3);
			obinFile.read(p.value.getRgba(),4);
			pa.addPoint(PointAccumulator::Point(p.getComponents()),PointAccumulator::Color(p.value.getRgba()));
			}
		}
	}

void loadPointFileOctree(PointAccumulator& pa,const char* fileNameStem)
	{
	/* Open the input octree structure and octree point files: */
	char octFileName[1024];
	snprintf(octFileName,sizeof(octFileName),"%s.oct",fileNameStem);
	IO::SeekableFilePtr octFile(IO::openSeekableFile(octFileName));
	octFile->setEndianness(Misc::LittleEndian);
	char obinFileName[1024];
	snprintf(obinFileName,sizeof(obinFileName),"%s.obin",fileNameStem);
	IO::SeekableFilePtr obinFile(IO::openSeekableFile(obinFileName));
	obinFile->setEndianness(Misc::LittleEndian);
	
	/* Read the octree structure file header: */
	OldLidarOctreeFileHeader ofh(*octFile);
	
	/* Read all original points from the octree: */
	readOctreeFileSubtree(pa,*octFile,*obinFile);
	}

/*****************************************************
Helper class to load LiDAR files in new octree format:
*****************************************************/

class LidarFileLoader
	{
	/* Elements: */
	private:
	PointAccumulator& pa;
	
	/* Constructors and destructors: */
	public:
	LidarFileLoader(PointAccumulator& sPa)
		:pa(sPa)
		{
		}
	
	/* Methods: */
	void operator()(LidarProcessOctree::Node& node,unsigned int nodeLevel)
		{
		/* Check if this node is a leaf: */
		if(node.isLeaf())
			{
			/* Add each node point to the point accumulator: */
			for(unsigned int i=0;i<node.getNumPoints();++i)
				{
				LidarPoint p=node[i];
				pa.addPoint(PointAccumulator::Point(p.getComponents()),PointAccumulator::Color(p.value.getRgba()));
				}
			}
		}
	};


void loadLidarFile(PointAccumulator& pa,const char* lidarFileName)
	{
	/* Open the LiDAR file: */
	LidarProcessOctree lpo(lidarFileName,size_t(64)*size_t(1024)*size_t(1024));
	LidarFileLoader lfl(pa);
	lpo.processNodesPostfix(lfl);
	}

/**************
Helper classes:
**************/

enum PointFileType // Enumerated type for point file formats
	{
	AUTO, // Tries to autodetect input file format based on file name extension
	TIFFDEM, // DEM file in TIFF format
	XYZBIL, // BIP/BIL/BSQ image containing 3D point coordinates in 3 channels
	BIN, // Simple binary format: number of points followed by (x, y, z, i) tuples
	BINRGB, // Simple binary format: number of points followed by (x, y, z, r, g, b) tuples
	PLY, // File in standard PLY format; only reads vertex positions and colors
	LAS, // Standard binary LiDAR point cloud interchange format
	XYZI, // Simple ASCII format: rows containing space-separated (x, y, z, i) tuples
	XYZRGB, // Simple ASCII format: rows containing space-separated (x, y, z, r, g, b) tuples
	ASCII, // Generic ASCII format with intensity data
	ASCIIRGB, // Generic ASCII format with RGB data
	CSV, // Strict comma-separated values file with intensity data
	CSVRGB, // Strict comma-separated values file with RGB data
	BLOCKEDASCII, // Blocked ASCII format with intensity data
	BLOCKEDASCIIRGB, // Blocked ASCII format with RGB data
	IDL, // Redshift data file in IDL format
	OCTREE, // Point octree file in old file format
	LIDAR, // LiDAR file in new octree file format
	ILLEGAL
	};

/****************
Helper functions:
****************/

bool readColumnIndexMask(int argc,char* argv[],int& argi,int columnIndices[6])
	{
	/* Initialize the column indices to invalid: */
	for(int i=0;i<6;++i)
		columnIndices[i]=-1;
	
	/* Read all parameters that are integers: */
	for(int i=0;i<6;++i)
		{
		++argi;
		if(argi<argc)
			{
			char* conversionEnd;
			int value=int(strtol(argv[argi],&conversionEnd,10));
			if(*conversionEnd=='\0')
				columnIndices[i]=value;
			else
				{
				--argi;
				break;
				}
			}
		}
	
	/* Check if the index array at least contains x, y, z components: */
	return columnIndices[0]>=0&&columnIndices[1]>=0&&columnIndices[2]>=0;
	}

int main(int argc,char* argv[])
	{
	/* Set default values for all parameters: */
	unsigned int memoryCacheSize=512;
	unsigned int tempOctreeMaxNumPointsPerNode=4096;
	std::string tempOctreeFileNameTemplate="/tmp/LidarPreprocessorTempOctree";
	unsigned int maxNumPointsPerNode=4096;
	int numThreads=1;
	std::string tempPointFileNameTemplate="/tmp/LidarPreprocessorTempPoints";
	
	try
		{
		/* Open LidarViewer's configuration file: */
		Misc::ConfigurationFile configFile(LIDARVIEWER_CONFIGFILENAME);
		Misc::ConfigurationFileSection cfg=configFile.getSection("/LidarPreprocessor");
		
		/* Override program settings from configuration file: */
		memoryCacheSize=cfg.retrieveValue<unsigned int>("./memoryCacheSize",memoryCacheSize);
		tempOctreeMaxNumPointsPerNode=cfg.retrieveValue<unsigned int>("./tempOctreeMaxNumPointsPerNode",tempOctreeMaxNumPointsPerNode);
		tempOctreeFileNameTemplate=cfg.retrieveValue<std::string>("./tempOctreeFileNameTemplate",tempOctreeFileNameTemplate);
		maxNumPointsPerNode=cfg.retrieveValue<unsigned int>("./maxNumPointsPerNode",maxNumPointsPerNode);
		numThreads=cfg.retrieveValue<int>("./numThreads",numThreads);
		tempPointFileNameTemplate=cfg.retrieveValue<std::string>("./tempPointFileNameTemplate",tempPointFileNameTemplate);
		}
	catch(const std::runtime_error&)
		{
		/* Just ignore the error */
		}
	
	/* Initialize transient parameters: */
	const char* outputFileName=0;
	PointFileType pointFileType=AUTO;
	unsigned int tiffImageIndex=0;
	int asciiColumnIndices[6];
	unsigned int lasClassMask=~0x0U;
	int numHeaderLines=0;
	const char* plyColorNames[3]=
		{
		"red","green","blue"
		};
	bool havePoints=false;
	
	/* Parse the command line and load all input files: */
	Misc::Timer loadTimer;
	PointAccumulator pa;
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"o")==0)
				{
				++i;
				if(i<argc)
					outputFileName=argv[i];
				else
					std::cerr<<"Dangling -o flag on command line"<<std::endl;
				}
			else if(strcasecmp(argv[i]+1,"np")==0)
				{
				++i;
				if(i<argc)
					maxNumPointsPerNode=(unsigned int)(atoi(argv[i]));
				else
					std::cerr<<"Dangling -np flag on command line"<<std::endl;
				}
			else if(strcasecmp(argv[i]+1,"nt")==0)
				{
				++i;
				if(i<argc)
					numThreads=atoi(argv[i]);
				else
					std::cerr<<"Dangling -nt flag on command line"<<std::endl;
				}
			else if(strcasecmp(argv[i]+1,"ooc")==0)
				{
				++i;
				if(i<argc)
					memoryCacheSize=(unsigned int)(atoi(argv[i]));
				else
					std::cerr<<"Dangling -ooc flag on command line"<<std::endl;
				}
			else if(strcasecmp(argv[i]+1,"to")==0)
				{
				++i;
				if(i<argc)
					{
					if(!havePoints)
						tempOctreeFileNameTemplate=argv[i];
					else
						std::cerr<<"Ignoring -to flag; must be specified before any input point sets are read"<<std::endl;
					}
				else
					std::cerr<<"Dangling -to flag on command line"<<std::endl;
				}
			else if(strcasecmp(argv[i]+1,"tp")==0)
				{
				++i;
				if(i<argc)
					tempPointFileNameTemplate=argv[i];
				else
					std::cerr<<"Dangling -tp flag on command line"<<std::endl;
				}
			else if(strcasecmp(argv[i]+1,"lasOffset")==0)
				{
				if(havePoints)
					{
					std::cerr<<"Ignoring lasOffset argument; must be specified before any input files are read"<<std::endl;
					i+=3;
					}
				else if(i+3<argc)
					{
					PointAccumulator::Vector newPointOffset;
					for(int j=0;j<3;++j)
						{
						++i;
						newPointOffset[j]=atof(argv[i]);
						}
					pa.setPointOffset(newPointOffset);
					}
				else
					std::cerr<<"Dangling -lasOffset flag on command line"<<std::endl;
				}
			else if(strcasecmp(argv[i]+1,"lasOffsetFile")==0)
				{
				++i;
				if(havePoints)
					std::cerr<<"Ignoring lasOffsetFile argument; must be specified before any input files are read"<<std::endl;
				else if(i<argc)
					{
					/* Read the point offset from a binary file: */
					try
						{
						IO::FilePtr offsetFile=IO::openFile(argv[i]);
						offsetFile->setEndianness(Misc::LittleEndian);
						PointAccumulator::Vector newPointOffset;
						offsetFile->read(newPointOffset.getComponents(),3);
						pa.setPointOffset(newPointOffset);
						}
					catch(const std::runtime_error& err)
						{
						/* Print a warning and carry on: */
						std::cerr<<"Ignoring lasOffsetFile argument due to error "<<err.what()<<" when reading file "<<argv[i]<<std::endl;
						}
					}
				else
					std::cerr<<"Dangling -lasOffsetFile flag on command line"<<std::endl;
				}
			else if(strcasecmp(argv[i]+1,"noLasOffset")==0)
				pa.resetPointOffset();
			else if(strcasecmp(argv[i]+1,"transform")==0)
				{
				++i;
				if(i<argc)
					{
					/* Set the point accumulator's current transformation: */
					pa.setTransform(Misc::ValueCoder<Geometry::OrthogonalTransformation<double,3> >::decode(argv[i],argv[i]+strlen(argv[i]),0));
					}
				else
					std::cerr<<"Dangling -transform flag on command line"<<std::endl;
				}
			else if(strcasecmp(argv[i]+1,"notransform")==0)
				pa.resetTransform();
			else if(strcasecmp(argv[i]+1,"c")==0)
				{
				if(i+3<argc)
					{
					float newColorMask[3];
					for(int j=0;j<3;++j)
						{
						++i;
						newColorMask[j]=float(atof(argv[i]));
						}
					pa.setColorMask(newColorMask);
					}
				else
					{
					i=argc;
					std::cerr<<"Dangling -c flag on command line"<<std::endl;
					}
				}
			else if(strcasecmp(argv[i]+1,"auto")==0)
				pointFileType=AUTO;
			else if(strcasecmp(argv[i]+1,"tiff")==0)
				pointFileType=TIFFDEM;
			else if(strcasecmp(argv[i]+1,"tiffImageIndex")==0)
				{
				if(i+1<argc)
					{
					++i;
					tiffImageIndex=atoi(argv[i]);
					}
				else
					{
					i=argc;
					std::cerr<<"Dangling -tiffImageIndex flag on command line"<<std::endl;
					}
				}
			else if(strcasecmp(argv[i]+1,"xyzbil")==0)
				pointFileType=XYZBIL;
			else if(strcasecmp(argv[i]+1,"bin")==0)
				pointFileType=BIN;
			else if(strcasecmp(argv[i]+1,"binrgb")==0)
				pointFileType=BINRGB;
			else if(strcasecmp(argv[i]+1,"ply")==0)
				pointFileType=PLY;
			else if(strcasecmp(argv[i]+1,"plyColorNames")==0)
				{
				if(i+3<argc)
					{
					for(int j=0;j<3;++j)
						{
						++i;
						plyColorNames[j]=argv[i];
						}
					}
				else
					{
					i=argc;
					std::cerr<<"Dangling -plyColorNames flag on command line"<<std::endl;
					}
				}
			else if(strcasecmp(argv[i]+1,"las")==0)
				pointFileType=LAS;
			else if(strcasecmp(argv[i]+1,"lasClasses")==0)
				{
				/* Build a bit mask of LAS point classes to process: */
				lasClassMask=0x0U;
				while(i+1<argc)
					{
					/* Parse the next argument as an unsigned integer: */
					unsigned int classBit=0U;
					char* cPtr;
					for(cPtr=argv[i+1];*cPtr>='0'&&*cPtr<='9';++cPtr)
						classBit=classBit*10+(unsigned int)(*cPtr-'0');
					
					/* Bail out if it wasn't: */
					if(*cPtr!='\0')
						break;
					
					/* Add the class bit and go to the next argument: */
					lasClassMask|=0x1U<<classBit;
					++i;
					}
				}
			else if(strcasecmp(argv[i]+1,"header")==0)
				{
				++i;
				numHeaderLines=atoi(argv[i]);
				}
			else if(strcasecmp(argv[i]+1,"xyzi")==0)
				pointFileType=XYZI;
			else if(strcasecmp(argv[i]+1,"xyzrgb")==0)
				pointFileType=XYZRGB;
			else if(strcasecmp(argv[i]+1,"ascii")==0)
				{
				pointFileType=ASCII;
				
				/* Read the column index mask: */
				if(!readColumnIndexMask(argc,argv,i,asciiColumnIndices))
					{
					std::cerr<<"Invalid column indices for ASCII file"<<std::endl;
					pointFileType=ILLEGAL;
					}
				}
			else if(strcasecmp(argv[i]+1,"asciirgb")==0)
				{
				pointFileType=ASCIIRGB;
				
				/* Read the column index mask: */
				if(!readColumnIndexMask(argc,argv,i,asciiColumnIndices))
					{
					std::cerr<<"Invalid column indices for RGB ASCII file"<<std::endl;
					pointFileType=ILLEGAL;
					}
				}
			else if(strcasecmp(argv[i]+1,"csv")==0)
				{
				pointFileType=CSV;
				
				/* Read the column index mask: */
				if(!readColumnIndexMask(argc,argv,i,asciiColumnIndices))
					{
					std::cerr<<"Invalid column indices for CSV file"<<std::endl;
					pointFileType=ILLEGAL;
					}
				}
			else if(strcasecmp(argv[i]+1,"csvrgb")==0)
				{
				pointFileType=CSVRGB;
				
				/* Read the column index mask: */
				if(!readColumnIndexMask(argc,argv,i,asciiColumnIndices))
					{
					std::cerr<<"Invalid column indices for RGB CSV file"<<std::endl;
					pointFileType=ILLEGAL;
					}
				}
			else if(strcasecmp(argv[i]+1,"blockedascii")==0)
				{
				pointFileType=BLOCKEDASCII;
				
				/* Read the column index mask: */
				if(!readColumnIndexMask(argc,argv,i,asciiColumnIndices))
					{
					std::cerr<<"Invalid column indices for blocked ASCII file"<<std::endl;
					pointFileType=ILLEGAL;
					}
				}
			else if(strcasecmp(argv[i]+1,"blockedasciirgb")==0)
				{
				pointFileType=BLOCKEDASCIIRGB;
				
				/* Read the column index mask: */
				if(!readColumnIndexMask(argc,argv,i,asciiColumnIndices))
					{
					std::cerr<<"Invalid column indices for blocked RGB ASCII file"<<std::endl;
					pointFileType=ILLEGAL;
					}
				}
			else if(strcasecmp(argv[i]+1,"idl")==0)
				pointFileType=IDL;
			else if(strcasecmp(argv[i]+1,"oct")==0)
				pointFileType=OCTREE;
			else if(strcasecmp(argv[i]+1,"lidar")==0)
				pointFileType=LIDAR;
			else
				std::cerr<<"Unrecognized command line option "<<argv[i]<<std::endl;
			}
		else
			{
			PointFileType thisPointFileType=pointFileType;
			if(thisPointFileType==AUTO)
				{
				/* Find the extension of the input file: */
				const char* extPtr=Misc::getExtension(argv[i]);
				
				/* Determine the file type: */
				if(strcasecmp(extPtr,".tif")==0||strcasecmp(extPtr,".tiff")==0)
					thisPointFileType=TIFFDEM;
				else if(strcasecmp(extPtr,".img")==0)
					thisPointFileType=XYZBIL;
				else if(strcasecmp(extPtr,".bin")==0)
					thisPointFileType=BIN;
				else if(strcasecmp(extPtr,".binrgb")==0)
					thisPointFileType=BINRGB;
				else if(strcasecmp(extPtr,".ply")==0)
					thisPointFileType=PLY;
				else if(strcasecmp(extPtr,".las")==0)
					thisPointFileType=LAS;
				else if(strcasecmp(extPtr,".xyzi")==0)
					thisPointFileType=XYZI;
				else if(strcasecmp(extPtr,".xyzrgb")==0)
					thisPointFileType=XYZI;
				else if(strcasecmp(extPtr,".oct")==0)
					thisPointFileType=OCTREE;
				else if(strcasecmp(extPtr,".LiDAR")==0)
					thisPointFileType=LIDAR;
				else
					thisPointFileType=ILLEGAL;
				}
			
			if(thisPointFileType!=ILLEGAL&&!havePoints)
				{
				/* Initialize the point accumulator: */
				pa.setMemorySize(memoryCacheSize,tempOctreeMaxNumPointsPerNode);
				pa.setTempOctreeFileNameTemplate(tempOctreeFileNameTemplate+"XXXXXX");
				}
			
			/* Reset the point accumulator's spatial and color extents: */
			pa.resetExtents();
			
			switch(thisPointFileType)
				{
				case TIFFDEM:
					{
					std::cout<<"Processing TIFF input file "<<argv[i]<<"..."<<std::flush;
					TIFFDEMLoader tiff(pa,argv[i],tiffImageIndex);
					tiff.collectPoints();
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
					}
				
				case XYZBIL:
					std::cout<<"Processing XYZ BIL input file "<<argv[i]<<"..."<<std::flush;
					loadXYZBILImage(pa,argv[i]);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				case BIN:
					std::cout<<"Processing binary input file "<<argv[i]<<"..."<<std::flush;
					loadPointFileBin(pa,argv[i]);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				case BINRGB:
					std::cout<<"Processing RGB binary input file "<<argv[i]<<"..."<<std::flush;
					loadPointFileBinRgb(pa,argv[i]);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				case PLY:
					std::cout<<"Processing PLY input file "<<argv[i]<<"..."<<std::flush;
					readPlyFile(pa,argv[i],plyColorNames);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				case LAS:
					std::cout<<"Processing binary input file "<<argv[i]<<"..."<<std::flush;
					loadPointFileLas(pa,argv[i],lasClassMask);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				case XYZI:
					std::cout<<"Processing XYZI input file "<<argv[i]<<"..."<<std::flush;
					loadPointFileXyzi(pa,argv[i]);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				case XYZRGB:
					std::cout<<"Processing XYZRGB input file "<<argv[i]<<"..."<<std::flush;
					loadPointFileXyzrgb(pa,argv[i]);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				case ASCII:
					std::cout<<"Processing generic ASCII input file "<<argv[i]<<"..."<<std::flush;
					loadPointFileGenericASCII(pa,argv[i],numHeaderLines,false,false,asciiColumnIndices);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				case ASCIIRGB:
					std::cout<<"Processing generic RGB ASCII input file "<<argv[i]<<"..."<<std::flush;
					loadPointFileGenericASCII(pa,argv[i],numHeaderLines,false,true,asciiColumnIndices);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				case CSV:
					std::cout<<"Processing generic CSV input file "<<argv[i]<<"..."<<std::flush;
					loadPointFileGenericASCII(pa,argv[i],numHeaderLines,true,false,asciiColumnIndices);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				case CSVRGB:
					std::cout<<"Processing generic RGB CSV input file "<<argv[i]<<"..."<<std::flush;
					loadPointFileGenericASCII(pa,argv[i],numHeaderLines,true,true,asciiColumnIndices);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				case BLOCKEDASCII:
					std::cout<<"Processing blocked ASCII input file "<<argv[i]<<"..."<<std::flush;
					loadPointFileBlockedASCII(pa,argv[i],numHeaderLines,false,asciiColumnIndices);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				case BLOCKEDASCIIRGB:
					std::cout<<"Processing blocked RGB ASCII input file "<<argv[i]<<"..."<<std::flush;
					loadPointFileBlockedASCII(pa,argv[i],numHeaderLines,true,asciiColumnIndices);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				case IDL:
					std::cout<<"Processing redshift IDL input file "<<argv[i]<<"..."<<std::flush;
					loadPointFileIdl(pa,argv[i]);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				case OCTREE:
					std::cout<<"Processing LiDAR octree input file "<<argv[i]<<"..."<<std::flush;
					loadPointFileOctree(pa,argv[i]);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				case LIDAR:
					std::cout<<"Processing LiDAR input file "<<argv[i]<<"..."<<std::flush;
					loadLidarFile(pa,argv[i]);
					havePoints=true;
					std::cout<<" done."<<std::endl;
					break;
				
				default:
					std::cerr<<"Input file "<<argv[i]<<" has an unrecognized file format"<<std::endl;
				}
			
			/* Print the spatial and color extents of the just-read input file: */
			pa.printExtents();
			}
		}
	
	/* Check if an output file name was given: */
	if(outputFileName==0)
		{
		std::cerr<<"Usage: "<<argv[0]<<" -o <output file name stem> [<option 1>] ... [<option n>] <input file spec 1> ... <input file spec n>"<<std::endl;
		std::cerr<<"Options: -np <max points per node>"<<std::endl;
		std::cerr<<"         -nt <number of threads>"<<std::endl;
		std::cerr<<"         -ooc <memory cache size in MB>"<<std::endl;
		std::cerr<<"         -to <temporary octree file name template>"<<std::endl;
		std::cerr<<"         -tp <temporary point file name template>"<<std::endl;
		std::cerr<<"         -lasOffset <offset x> <offset y> <offset z>"<<std::endl;
		std::cerr<<"         -lasOffsetFile <binary offset file name>"<<std::endl;
		std::cerr<<"         -noLasOffset"<<std::endl;
		std::cerr<<"         -plyColorNames <red component name> <green component name> <blue component name>"<<std::endl;
		std::cerr<<"         -transform <orthogonal transformation specification>"<<std::endl;
		std::cerr<<"Input file spec: [-c <red> <green> <blue>] [-header <number of header lines>] <format spec> <file name>"<<std::endl;
		std::cerr<<"Format spec: -AUTO"<<std::endl;
		std::cerr<<"             -BIN"<<std::endl;
		std::cerr<<"             -BINRGB"<<std::endl;
		std::cerr<<"             -PLY"<<std::endl;
		std::cerr<<"             -LAS"<<std::endl;
		std::cerr<<"             -XYZI"<<std::endl;
		std::cerr<<"             -XYZRGB"<<std::endl;
		std::cerr<<"             -ASCII <x column> <y column> <z column> [<intensity column>]"<<std::endl;
		std::cerr<<"             -ASCIIRGB <x column> <y column> <z column> [<r column> <g column> <b column>]"<<std::endl;
		std::cerr<<"             -CSV <x column> <y column> <z column> [<intensity column>]"<<std::endl;
		std::cerr<<"             -CSVRGB <x column> <y column> <z column> [<r column> <g column> <b column>]"<<std::endl;
		std::cerr<<"             -BLOCKEDASCII <x column> <y column> <z column> [<intensity column>]"<<std::endl;
		std::cerr<<"             -BLOCKEDASCIIRGB <x column> <y column> <z column> [<r column> <g column> <b column>]"<<std::endl;
		std::cerr<<"             -IDL"<<std::endl;
		std::cerr<<"             -OCT"<<std::endl;
		std::cerr<<"             -LIDAR"<<std::endl;
		return 1;
		}
	
	/* Finish reading points: */
	pa.finishReading();
	loadTimer.elapse();
	
	/* Construct an octree with less than maxPointsPerNode points per leaf: */
	Misc::Timer createTimer;
	LidarOctreeCreator tree(pa.getMaxNumCacheablePoints(),maxNumPointsPerNode,numThreads,pa.getTempOctrees(),tempPointFileNameTemplate+"XXXXXX");
	
	/* Delete the temporary point octrees: */
	pa.deleteTempOctrees();
	createTimer.elapse();
	
	/* Write the octree structure and data to the destination LiDAR file: */
	Misc::Timer writeTimer;
	tree.write(size_t(memoryCacheSize)*size_t(1024*1024),outputFileName);
	writeTimer.elapse();
	
	/* Check if a point offset was defined: */
	if(pa.getPointOffset()!=PointAccumulator::Vector::zero)
		{
		/* Write the point offsets to an offset file: */
		std::string offsetFileName=outputFileName;
		offsetFileName.append("/Offset");
		IO::FilePtr offsetFile(IO::openFile(offsetFileName.c_str(),IO::File::WriteOnly));
		offsetFile->setEndianness(Misc::LittleEndian);
		offsetFile->write(pa.getPointOffset().getComponents(),3);
		}
	
	std::cout<<"Time to load input data: "<<loadTimer.getTime()<<"s, time to create octree: "<<createTimer.getTime()<<"s, time to write final octree files: "<<writeTimer.getTime()<<"s"<<std::endl;
	
	return 0;
	}
