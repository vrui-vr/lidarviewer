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

#ifndef PLYFILE_INCLUDED
#define PLYFILE_INCLUDED

#include <stddef.h>
#include <string>
#include <vector>
#include <Misc/Autopointer.h>

/* Forward declarations: */
namespace IO {
class File;
typedef Misc::Autopointer<File> FilePtr;
class ValueSource;
}

class PlyFile
	{
	/* Embedded classes: */
	public:
	class Value // Base class for values that can be read from PLY files
		{
		/* Constructors and destructors: */
		public:
		virtual ~Value(void);
		
		/* Methods: */
		virtual void skip(IO::File& binaryPlyFile) =0; // Skips one value in the given binary PLY file
		virtual void read(IO::File& binaryPlyFile) =0; // Reads a value from the given binary PLY file
		virtual void skip(IO::ValueSource& asciiPlyFile) =0; // Skips one value in the given ASCII PLY file
		virtual void read(IO::ValueSource& asciiPlyFile) =0; // Reads a value from the given ASCII PLY file
		};
	
	typedef std::vector<Value*> ValueList; // Type for lists of pointers to values
	
	enum AtomicType // Enumerated type to identify atomic data types in PLY files
		{
		SInt8,UInt8,SInt16,UInt16,SInt32,UInt32,Float32,Float64
		};
	
	class AtomicValue:public Value // Abstract base class to read atomic data values from PLY files
		{
		/* New methods: */
		public:
		virtual AtomicValue* clone(void) const =0; // Returns an identical copy of this atomic data value
		virtual AtomicType getType(void) const =0; // Returns the type of this atomic data value
		virtual bool isIntegral(void) const =0; // Returns true if this atomic data value has an integer type
		virtual bool isSigned(void) const =0; // Returns true if this atomic data value has a signed integer type
		virtual size_t getFileSize(void) const =0; // Returns the size of an atomic value of this type in a binary PLY file
		virtual size_t getMemorySize(void) const =0; // Returns the size of an atomic value of this type in memory
		virtual int getInt(void) const =0; // Returns the most-recently read value as a signed integer
		virtual unsigned int getUnsignedInt(void) const =0; // Returns the most-recently read value as an unsigned integer
		virtual double getDouble(void) const =0; // Returns the most-recently read value as a floating-point number
		};
	
	class ListValue:public Value // Abstract base class to read list data values from PLY files
		{
		/* New methods: */
		public:
		virtual bool areItemsIntegral(void) const =0; // Returns true if the items of this list data value have an integer type
		virtual bool areItemsSigned(void) const =0; // Returns true if the items of this list data value have a signed integer type
		virtual size_t getNumItems(void) const =0; // Returns the number of items in the most-recently read list value
		virtual int getInt(size_t index) const =0; // Returns the item of the given index in the most-recently read value as a signed integer
		virtual unsigned int getUnsignedInt(size_t index) const =0; // Returns the item of the given index in the most-recently read value as an unsigned integer
		virtual double getDouble(size_t index) const =0; // Returns the item of the given index in the most-recently read value as a floating-point number
		};
	
	class Property // Class for properties, which can describe either an atomic value, or a dynamic list of atomic values
		{
		/* Embedded classes: */
		public:
		enum Type
			{
			Atomic,List
			};
		
		/* Elements: */
		private:
		Type type; // Type of this property
		union
			{
			AtomicType atomicType; // Atomic data type for atomic properties
			struct
				{
				AtomicType numItemsType; // Atomic data type for list sizes for list properties
				AtomicType itemType; // Data type for list elements for list properties
				} list;
			} types; // The atomic type(s) defining this property
		std::string name; // The property's name
		
		/* Private methods: */
		static AtomicType parseDataType(const std::string& tag); // Parses an atomic data type from a string
		
		/* Constructors and destructors: */
		public:
		Property(IO::ValueSource& plyFileHeader); // Creates a property by reading from a PLY file header
		
		/* Methods: */
		Type getType(void) const // Returns the type of this property
			{
			return type;
			}
		bool isAtomic(void) const // Returns true if the property is atomic
			{
			return type==Atomic;
			}
		bool isList(void) const // Returns true if the property is a list
			{
			return type==List;
			}
		const std::string& getName(void) const // Returns the property's name
			{
			return name;
			}
		Value* createValue(void) const; // Returns a new value object to read this property from a PLY file
		};
	
	typedef std::vector<Property> PropertyList; // Type for lists of properties
	
	class Element // Class for elements, which are a list of values each having a list of properties
		{
		/* Elements: */
		private:
		std::string name; // The element's name
		size_t numValues; // The number of values comprising this element
		PropertyList properties; // The element's properties
		ValueList values; // The list of values used to read the element's properties
		
		/* Constructors and destructors: */
		public:
		Element(const std::string& sName,size_t sNumValues); // Creates an element with the given name and number of values and an empty property list
		private:
		Element(const Element& source); // Prohibit copy constructor
		public:
		Element(Element&& source); // Move constructor
		~Element(void); // Destroys the element
		
		/* Methods: */
		void appendProperty(IO::ValueSource& plyFileHeader); // Appends a property to the elemens's property list by reading from the given PLY file header
		const std::string& getName(void) const // Returns the element's name
			{
			return name;
			}
		bool isElement(const char* elementName) const // Returns true if the element's name matches the given string
			{
			return name==elementName;
			}
		size_t getNumValues(void) const // Returns the number of values comprising this element
			{
			return numValues;
			}
		const PropertyList& getProperties(void) const // Returns the element's property list
			{
			return properties;
			}
		bool hasListProperties(void) const; // Returns true if the element has at least one list property
		size_t findProperty(const char* propertyName) const; // Returns the index of the property of the given name within this element; throws an exception if the element does not contain the property
		const Property& getProperty(size_t propertyIndex) const // Returns the property of the given index
			{
			return properties[propertyIndex];
			}
		const Value& getPropertyValue(size_t propertyIndex) const // Returns the most-recently read value for the property of the given index
			{
			return *(values[propertyIndex]);
			}
		const AtomicValue& getAtomicPropertyValue(size_t propertyIndex) const; // Returns the most-recently read value for the property of the given index as an atomic value; throws an exception if the property is not of an atomic type
		const ListValue& getListPropertyValue(size_t propertyIndex) const; // Returns the most-recently read value for the property of the given index as a list value; throws an exception if the property is not of a list type
		template <class FileParam>
		void skip(FileParam& file) // Skips the element's next value from a PLY file in binary or ASCII format
			{
			/* Skip the next value's properties: */
			for(ValueList::iterator vIt=values.begin();vIt!=values.end();++vIt)
				(*vIt)->skip(file);
			}
		template <class FileParam>
		void read(FileParam& file) // Reads the element's next value from a PLY file in binary or ASCII format
			{
			/* Read the next value's properties: */
			for(ValueList::iterator vIt=values.begin();vIt!=values.end();++vIt)
				(*vIt)->read(file);
			}
		};
	
	typedef std::vector<Element> ElementList; // Type for lists of elements
	
	/* Elements: */
	private:
	IO::FilePtr file; // Pointer to the underlying file
	IO::ValueSource* asciiBody; // Pointer to a value source to read an ASCII PLY file; null if the PLY file is in binary format
	ElementList elements; // List of the elements contained in the PLY file
	ElementList::iterator current; // Iterator to the element that is currently being read from the PLY file
	size_t numValuesLeft; // Number of values left to read in the current element
	
	/* Constructors and destructors: */
	public:
	PlyFile(IO::File& sFile); // Creates a PLY file around the given file and reads that file's PLY header
	~PlyFile(void); // Destroys the PLY file
	
	/* Methods: */
	const ElementList& getElements(void) const // Returns the list of elements contained in the PLY file
		{
		return elements;
		}
	bool eof(void) const // Returns true after all values of all elements have been read
		{
		return current==elements.end();
		}
	const Element& getCurrent(void) const // Returns the element that is currently being read from the PLY file; must not be called after eof has returned true
		{
		return *current;
		}
	bool eoe(void) const // Returns true if all the current element's values have been read; call nextElement immediately after eoe returned true to advance to the next element
		{
		return numValuesLeft==0;
		}
	void nextElement(void); // Start reading the next element after eoe has returned true; must not be called if eoe did not return true immediately before
	void read(void); // Reads the next value of the current element; must not be called after eoe has returned true and before nextElement has been called
	void skipElement(void); // Skips all remaining values of the current element and starts reading the next element
	};

#endif
