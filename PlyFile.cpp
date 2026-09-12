/***********************************************************************
PlyFile - Class representing structured files in PLY format, typically
used to represent 3D geometry.
Copyright (c) 2004-2026 Oliver Kreylos

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

#include "PlyFile.h"

#include <Misc/SizedTypes.h>
#include <Misc/StdError.h>
#include <IO/File.h>
#include <IO/ValueSource.h>

/*******************************
Methods of class PlyFile::Value:
*******************************/

PlyFile::Value::~Value(void)
	{
	}

namespace {

/**********************************************
Templatized class to define atomic type traits:
**********************************************/

template <int dataTypeParam>
class AtomicValueTraits
	{
	};

/************************************************
Template specializations for actual atomic types:
************************************************/

template <>
class AtomicValueTraits<PlyFile::SInt8>
	{
	/* Embedded classes: */
	public:
	static const int dataType=PlyFile::SInt8;
	static const bool isIntegral=true;
	static const bool isSigned=true;
	typedef Misc::SInt8 FileType;
	typedef int MemoryType;
	};

template <>
class AtomicValueTraits<PlyFile::UInt8>
	{
	/* Embedded classes: */
	public:
	static const int dataType=PlyFile::UInt8;
	static const bool isIntegral=true;
	static const bool isSigned=false;
	typedef Misc::UInt8 FileType;
	typedef unsigned int MemoryType;
	};

template <>
class AtomicValueTraits<PlyFile::SInt16>
	{
	/* Embedded classes: */
	public:
	static const int dataType=PlyFile::SInt16;
	static const bool isIntegral=true;
	static const bool isSigned=true;
	typedef Misc::SInt16 FileType;
	typedef int MemoryType;
	};

template <>
class AtomicValueTraits<PlyFile::UInt16>
	{
	/* Embedded classes: */
	public:
	static const int dataType=PlyFile::UInt16;
	static const bool isIntegral=true;
	static const bool isSigned=false;
	typedef Misc::UInt16 FileType;
	typedef unsigned int MemoryType;
	};

template <>
class AtomicValueTraits<PlyFile::SInt32>
	{
	/* Embedded classes: */
	public:
	static const int dataType=PlyFile::SInt32;
	static const bool isIntegral=true;
	static const bool isSigned=true;
	typedef Misc::SInt32 FileType;
	typedef int MemoryType;
	};

template <>
class AtomicValueTraits<PlyFile::UInt32>
	{
	/* Embedded classes: */
	public:
	static const int dataType=PlyFile::UInt32;
	static const bool isIntegral=true;
	static const bool isSigned=false;
	typedef Misc::UInt32 FileType;
	typedef unsigned int MemoryType;
	};

template <>
class AtomicValueTraits<PlyFile::Float32>
	{
	/* Embedded classes: */
	public:
	static const int dataType=PlyFile::Float32;
	static const bool isIntegral=false;
	static const bool isSigned=true;
	typedef Misc::Float32 FileType;
	typedef double MemoryType;
	};

template <>
class AtomicValueTraits<PlyFile::Float64>
	{
	/* Embedded classes: */
	public:
	static const int dataType=PlyFile::Float64;
	static const bool isIntegral=false;
	static const bool isSigned=true;
	typedef Misc::Float64 FileType;
	typedef double MemoryType;
	};

/*******************************************************************
Templatized helper class to read atomic values from ASCII PLY files:
*******************************************************************/

template <typename MemoryTypeParam>
class AsciiFileReader
	{
	};

template <>
class AsciiFileReader<int>
	{
	/* Methods: */
	public:
	static int readValue(IO::ValueSource& asciiFile)
		{
		return asciiFile.readInteger();
		}
	};

template <>
class AsciiFileReader<unsigned int>
	{
	/* Methods: */
	public:
	static unsigned int readValue(IO::ValueSource& asciiFile)
		{
		return asciiFile.readUnsignedInteger();
		}
	};

template <>
class AsciiFileReader<double>
	{
	/* Methods: */
	public:
	static double readValue(IO::ValueSource& asciiFile)
		{
		return asciiFile.readNumber();
		}
	};

/***************************************************************************
Templatized class to read atomic data values from ASCII or binary PLY files:
***************************************************************************/

template <PlyFile::AtomicType dataTypeParam>
class AtomicValueTemplate:public PlyFile::AtomicValue
	{
	/* Embedded classes: */
	public:
	typedef PlyFile::AtomicValue Base;
	typedef typename AtomicValueTraits<dataTypeParam>::FileType ValueFileType;
	typedef typename AtomicValueTraits<dataTypeParam>::MemoryType ValueMemoryType;
	
	/* Elements: */
	private:
	ValueMemoryType value; // The most recently read atomic data value
	
	/* Methods: */
	virtual void skip(IO::File& binaryPlyFile)
		{
		binaryPlyFile.skip<ValueFileType>(1);
		}
	virtual void read(IO::File& binaryPlyFile)
		{
		value=ValueMemoryType(binaryPlyFile.read<ValueFileType>());
		}
	virtual void skip(IO::ValueSource& asciiPlyFile)
		{
		/* We can't really skip in ASCII files, so we'll read and ignore: */
		AsciiFileReader<ValueMemoryType>::readValue(asciiPlyFile);
		}
	virtual void read(IO::ValueSource& asciiPlyFile)
		{
		value=AsciiFileReader<ValueMemoryType>::readValue(asciiPlyFile);
		}
	virtual PlyFile::AtomicValue* clone(void) const
		{
		return new AtomicValueTemplate(*this);
		}
	virtual PlyFile::AtomicType getType(void) const
		{
		return dataTypeParam;
		}
	virtual bool isIntegral(void) const
		{
		return AtomicValueTraits<dataTypeParam>::isIntegral;
		}
	virtual bool isSigned(void) const
		{
		return AtomicValueTraits<dataTypeParam>::isSigned;
		}
	virtual size_t getFileSize(void) const
		{
		return sizeof(ValueFileType);
		}
	virtual size_t getMemorySize(void) const
		{
		return sizeof(ValueMemoryType);
		}
	virtual int getInt(void) const
		{
		return int(value);
		}
	virtual unsigned int getUnsignedInt(void) const
		{
		return (unsigned int)(value);
		}
	virtual double getDouble(void) const
		{
		return double(value);
		}
	};

/********************************************
Factory class to create atomic value readers:
********************************************/

class AtomicValueFactory
	{
	/* Methods: */
	public:
	static PlyFile::AtomicValue* create(PlyFile::AtomicType dataType)
		{
		switch(dataType)
			{
			case PlyFile::SInt8:
				return new AtomicValueTemplate<PlyFile::SInt8>;
			
			case PlyFile::UInt8:
				return new AtomicValueTemplate<PlyFile::UInt8>;
			
			case PlyFile::SInt16:
				return new AtomicValueTemplate<PlyFile::SInt16>;
			
			case PlyFile::UInt16:
				return new AtomicValueTemplate<PlyFile::UInt16>;
			
			case PlyFile::SInt32:
				return new AtomicValueTemplate<PlyFile::SInt32>;
			
			case PlyFile::UInt32:
				return new AtomicValueTemplate<PlyFile::UInt32>;
			
			case PlyFile::Float32:
				return new AtomicValueTemplate<PlyFile::Float32>;
			
			case PlyFile::Float64:
				return new AtomicValueTemplate<PlyFile::Float64>;
			
			default:
				throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Invalid atomic type %d",dataType);
			}
		}
	};

}

namespace {

/*************************************************************************
Templatized class to read list data values from ASCII or binary PLY files:
*************************************************************************/

template <int numItemsTypeParam,int itemTypeParam>
class ListValueTemplate:public PlyFile::ListValue
	{
	/* Embedded classes: */
	public:
	typedef PlyFile::ListValue Base;
	typedef typename AtomicValueTraits<numItemsTypeParam>::FileType NumItemsFileType;
	typedef typename AtomicValueTraits<numItemsTypeParam>::MemoryType NumItemsMemoryType;
	typedef typename AtomicValueTraits<itemTypeParam>::FileType ItemFileType;
	typedef typename AtomicValueTraits<itemTypeParam>::MemoryType ItemMemoryType;
	typedef std::vector<ItemMemoryType> ItemList; // Type for lists of items
	
	/* Elements: */
	private:
	ItemList items; // The most-recently read list of items
	
	/* Methods from class Value: */
	public:
	virtual void skip(IO::File& binaryPlyFile)
		{
		/* Read the number of list items: */
		size_t numItems=NumItemsMemoryType(binaryPlyFile.read<NumItemsFileType>());
		
		/* Skip the list items: */
		binaryPlyFile.skip<ItemFileType>(numItems);
		}
	virtual void read(IO::File& binaryPlyFile)
		{
		/* Read the number of list items: */
		size_t numItems=NumItemsMemoryType(binaryPlyFile.read<NumItemsFileType>());
		
		/* Read all list items: */
		items.clear();
		items.reserve(numItems);
		for(size_t i=0;i<numItems;++i)
			items.push_back(ItemMemoryType(binaryPlyFile.read<ItemFileType>()));
		}
	virtual void skip(IO::ValueSource& asciiPlyFile)
		{
		/* Read the number of list items: */
		size_t numItems=AsciiFileReader<NumItemsMemoryType>::readValue(asciiPlyFile);
		
		/* We can't really skip in ASCII files, so we'll read all list items and ignore them: */
		for(size_t i=0;i<numItems;++i)
			AsciiFileReader<ItemMemoryType>::readValue(asciiPlyFile);
		}
	virtual void read(IO::ValueSource& asciiPlyFile)
		{
		/* Read the number of list items: */
		size_t numItems=AsciiFileReader<NumItemsMemoryType>::readValue(asciiPlyFile);
		
		/* Read all list items: */
		items.clear();
		items.reserve(numItems);
		for(size_t i=0;i<numItems;++i)
			items.push_back(AsciiFileReader<ItemMemoryType>::readValue(asciiPlyFile));
		}
	
	/* Methods from class ListValue: */
	virtual bool areItemsIntegral(void) const
		{
		return AtomicValueTraits<itemTypeParam>::isIntegral;
		}
	virtual bool areItemsSigned(void) const
		{
		return AtomicValueTraits<itemTypeParam>::isSigned;
		}
	virtual size_t getNumItems(void) const
		{
		return items.size();
		}
	virtual int getInt(size_t index) const
		{
		return int(items[index]);
		}
	virtual unsigned int getUnsignedInt(size_t index) const
		{
		return (unsigned int)(items[index]);
		}
	virtual double getDouble(size_t index) const
		{
		return double(items[index]);
		}
	};

/******************************************
Factory class to create list value readers:
******************************************/

class ListValueFactory
	{
	template <int numItemsTypeParam>
	class Inner
		{
		/* Methods: */
		public:
		static PlyFile::ListValue* create(int itemType)
			{
			switch(itemType)
				{
				case PlyFile::SInt8:
					return new ListValueTemplate<numItemsTypeParam,PlyFile::SInt8>;
				
				case PlyFile::UInt8:
					return new ListValueTemplate<numItemsTypeParam,PlyFile::UInt8>;
				
				case PlyFile::SInt16:
					return new ListValueTemplate<numItemsTypeParam,PlyFile::SInt16>;
				
				case PlyFile::UInt16:
					return new ListValueTemplate<numItemsTypeParam,PlyFile::UInt16>;
				
				case PlyFile::SInt32:
					return new ListValueTemplate<numItemsTypeParam,PlyFile::SInt32>;
				
				case PlyFile::UInt32:
					return new ListValueTemplate<numItemsTypeParam,PlyFile::UInt32>;
				
				case PlyFile::Float32:
					return new ListValueTemplate<numItemsTypeParam,PlyFile::Float32>;
				
				case PlyFile::Float64:
					return new ListValueTemplate<numItemsTypeParam,PlyFile::Float64>;
				
				default:
					throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Invalid list item type %d",itemType);
				}
			}
		};
	
	/* Methods: */
	public:
	static PlyFile::ListValue* create(PlyFile::AtomicType numItemsType,PlyFile::AtomicType itemType)
		{
		switch(numItemsType)
			{
			case PlyFile::SInt8:
				return Inner<PlyFile::SInt8>::create(itemType);
			
			case PlyFile::UInt8:
				return Inner<PlyFile::UInt8>::create(itemType);
			
			case PlyFile::SInt16:
				return Inner<PlyFile::SInt16>::create(itemType);
			
			case PlyFile::UInt16:
				return Inner<PlyFile::UInt16>::create(itemType);
			
			case PlyFile::SInt32:
				return Inner<PlyFile::SInt32>::create(itemType);
			
			case PlyFile::UInt32:
				return Inner<PlyFile::UInt32>::create(itemType);
			
			case PlyFile::Float32:
				return Inner<PlyFile::Float32>::create(itemType);
			
			case PlyFile::Float64:
				return Inner<PlyFile::Float64>::create(itemType);
			
			default:
				throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Invalid list counter type %d",numItemsType);
			}
		}
	};

}

/**********************************
Methods of class PlyFile::Property:
**********************************/

PlyFile::AtomicType PlyFile::Property::parseDataType(const std::string& tag)
	{
	static const char* dataTypeTags[2][8]=
		{
		{"char","uchar","short","ushort","int","uint","float","double"},
		{"int8","uint8","int16","uint16","int32","uint32","float32","float64"}
		};
	static const AtomicType dataTypes[8]=
		{
		SInt8,UInt8,SInt16,UInt16,SInt32,UInt32,Float32,Float64
		};
	int i;
	for(i=0;i<8;++i)
		if(tag==dataTypeTags[0][i]||tag==dataTypeTags[1][i])
			break;
	if(i>=8)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Unknown data type %s",tag.c_str());
	return dataTypes[i];
	}

PlyFile::Property::Property(IO::ValueSource& plyFile)
	{
	/* Read the property type: */
	std::string tag=plyFile.readString();
	
	if(tag=="list")
		{
		/* Parse a list property: */
		type=List;
		types.list.numItemsType=parseDataType(plyFile.readString());
		types.list.itemType=parseDataType(plyFile.readString());
		}
	else
		{
		/* Parse an atomic property: */
		type=Atomic;
		types.atomicType=parseDataType(tag);
		}
	
	/* Read the property name: */
	name=plyFile.readString();
	}

PlyFile::Value* PlyFile::Property::createValue(void) const
	{
	if(type==Atomic)
		return AtomicValueFactory::create(types.atomicType);
	else
		return ListValueFactory::create(types.list.numItemsType,types.list.itemType);
	}

/*********************************
Methods of class PlyFile::Element:
*********************************/

PlyFile::Element::Element(const std::string& sName,size_t sNumValues)
	:name(sName),numValues(sNumValues)
	{
	}

PlyFile::Element::Element(Element&& source)
	:name(std::move(source.name)),numValues(source.numValues),
	 properties(std::move(source.properties)),values(std::move(source.values))
	{
	}

PlyFile::Element::~Element(void)
	{
	/* Destroy all values: */
	for(ValueList::iterator vIt=values.begin();vIt!=values.end();++vIt)
		delete *vIt;
	}

void PlyFile::Element::appendProperty(IO::ValueSource& plyFileHeader)
	{
	/* Parse a new property from the given file header and store it: */
	properties.push_back(Property(plyFileHeader));
	
	/* Create a value to read the new property and store it: */
	values.push_back(properties.back().createValue());
	}

bool PlyFile::Element::hasListProperties(void) const
	{
	bool result=false;
	for(PropertyList::const_iterator pIt=properties.begin();!result&&pIt!=properties.end();++pIt)
		result=pIt->isList();
	
	return result;
	}

size_t PlyFile::Element::findProperty(const char* propertyName) const
	{
	size_t result=0;
	for(PropertyList::const_iterator pIt=properties.begin();pIt!=properties.end();++pIt,++result)
		if(pIt->getName()==propertyName)
			return result;
	
	throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Property %s not found",propertyName);
	}

const PlyFile::AtomicValue& PlyFile::Element::getAtomicPropertyValue(size_t propertyIndex) const
	{
	if(!properties[propertyIndex].isAtomic())
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Property %s is not an atomic property",properties[propertyIndex].getName().c_str());
	
	return *static_cast<const AtomicValue*>(values[propertyIndex]);
	}

const PlyFile::ListValue& PlyFile::Element::getListPropertyValue(size_t propertyIndex) const
	{
	if(!properties[propertyIndex].isList())
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Property %s is not a list property",properties[propertyIndex].getName().c_str());
	
	return *static_cast<const ListValue*>(values[propertyIndex]);
	}

/************************
Methods of class PlyFile:
************************/

PlyFile::PlyFile(IO::File& sFile)
	:file(&sFile),asciiBody(0)
	{
	/* Attach a new value source to the PLY file to read its header: */
	IO::ValueSource header(sFile);
	header.setPunctuation("\r\n");
	header.skipWs();
	
	/* Process the PLY file header: */
	ElementList::iterator currentElement=elements.end();
	bool isPly=false;
	int fileType=-1;
	Misc::Endianness endianness=Misc::HostEndianness;
	bool haveEndHeader=false;
	while(!header.eof())
		{
		/* Read the next tag: */
		std::string tag=header.readString();
		if(tag=="ply")
			isPly=true;
		else if(tag=="format")
			{
			/* Read the format type and version number: */
			std::string format=header.readString();
			if(format=="ascii")
				fileType=0;
			else if(format=="binary_little_endian")
				{
				fileType=1;
				endianness=Misc::LittleEndian;
				}
			else if(format=="binary_big_endian")
				{
				fileType=1;
				endianness=Misc::BigEndian;
				}
			else
				{
				/* Unknown format; bail out: */
				break;
				}
			double version=header.readNumber();
			if(version!=1.0)
				break;
			}
		else if(tag=="element")
			{
			/* Read the element name and number of values: */
			std::string elementName=header.readString();
			size_t numValues=header.readUnsignedInteger();
			
			/* Append a new element: */
			elements.push_back(Element(elementName,numValues));
			currentElement=elements.end()-1;
			}
		else if(tag=="property")
			{
			/* Bail out if there is no current element: */
			if(currentElement==elements.end())
				break;
			
			/* Append the property to the current element: */
			currentElement->appendProperty(header);
			}
		else if(tag=="end_header")
			{
			/* Skip the line break after the end_header tag: */
			header.skipLine();
			
			/* Bail out: */
			haveEndHeader=true;
			break;
			}
		
		/* Skip the unknown or ignored tag: */
		header.skipLine();
		header.skipWs();
		}
	
	/* Check if the header was read completely: */
	if(!isPly||!haveEndHeader||fileType<0)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"File is not a valid version 1.0 PLY file");
	
	/* Check the file's type: */
	if(fileType==0)
		{
		/* Attach a value source to the PLY file's body: */
		asciiBody=new IO::ValueSource(*file);
		}
	else
		{
		/* Set the PLY file's endianness: */
		file->setEndianness(endianness);
		}
	
	/* Initialize the reading state: */
	current=elements.begin();
	if(current!=elements.end())
		numValuesLeft=current->getNumValues();
	}

PlyFile::~PlyFile(void)
	{
	/* Destroy the potential value source that was used to read the body of an ASCII PLY file: */
	delete asciiBody;
	}

void PlyFile::nextElement(void)
	{
	/* Advance the current element and check if there is another element: */
	++current;
	if(current!=elements.end())
		numValuesLeft=current->getNumValues();
	}

void PlyFile::read(void)
	{
	/* Read the next value into the current element and reduce the leftover count: */
	if(asciiBody!=0)
		current->read(*asciiBody);
	else
		current->read(*file);
	--numValuesLeft;
	}

void PlyFile::skipElement(void)
	{
	/* Skip all remaining values in the current element: */
	while(numValuesLeft>0)
		{
		if(asciiBody!=0)
			current->skip(*asciiBody);
		else
			current->skip(*file);
		--numValuesLeft;
		}
	
	/* Advance the current element and check if there is another element: */
	++current;
	if(current!=elements.end())
		numValuesLeft=current->getNumValues();
	}
