/**
 ****************************************************************************
 * <P> XML.c - implementation file for basic XML parser written in ANSI C++
 * for portability. It works by using recursion and a node tree for breaking
 * down the elements of an XML document.  </P>
 *
 * @version     V1.08
 *
 * @author      Frank Vanden Berghen
 * based on original implementation by Martyn C Brown
 *
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ****************************************************************************
 */
#ifndef __INCLUDE_XML_NODE__
#define __INCLUDE_XML_NODE__

#include "Util/NonCopyable.hpp"
#include "Util/tstring.hpp"
#include "Compiler.h"

#include <list>
#include <forward_list>

#include <assert.h>
#include <tchar.h>

class TextWriter;

/** Enumeration for XML parse errors. */
enum XMLError {
  eXMLErrorNone = 0,
  eXMLErrorEmpty,
  eXMLErrorFirstNotStartTag,
  eXMLErrorMissingTagName,
  eXMLErrorMissingEndTagName,
  eXMLErrorNoMatchingQuote,
  eXMLErrorUnmatchedEndTag,
  eXMLErrorUnexpectedToken,
  eXMLErrorInvalidTag,
  eXMLErrorNoElements,
  eXMLErrorFileNotFound
};

/** Structure used to obtain error details if the parse fails. */
struct XMLResults {
  enum XMLError error;
  unsigned nLine, nColumn;
};

struct XML;

struct XMLNode {
//  friend class XMLNode;
protected:
  /**
   * To allow shallow copy and "intelligent/smart" pointers (automatic
   * delete).
   */
  struct XMLNodeData : private NonCopyable {
    /** Structure for XML attribute. */
    struct Attribute {
      const TCHAR *lpszName;
      const TCHAR *lpszValue;
    };

    /** Element name (=NULL if root) */
    const TCHAR *lpszName;

    /** Whether node is an XML declaration - '<?xml ?>' */
    bool isDeclaration;

    /** Array of child nodes */
    std::list<XMLNode> pChild;

    /** A concatentation of all text nodes */
    tstring text;

    /** Array of attributes */
    std::forward_list<Attribute> pAttribute;

    unsigned ref_count;

    XMLNodeData(const TCHAR *_name, bool _is_declaration)
      :lpszName(_name),
       isDeclaration(_is_declaration),
       ref_count(1) {}
    ~XMLNodeData();

    void Ref();
    void Unref();

    bool HasChildren() const {
      return !pChild.empty() || !text.empty();
    }

    void AddAttribute(const TCHAR *name, const TCHAR *value) {
      pAttribute.push_front((Attribute){name, value});
    }

    typedef std::list<XMLNode>::const_iterator const_iterator;

    const_iterator begin() const {
      return pChild.begin();
    }

    const_iterator end() const {
      return pChild.end();
    }
  };

  XMLNodeData *d;

  /**
   * Protected constructor: use "parse" functions to get your first
   * instance of XMLNode.
   */
  XMLNode(const TCHAR *lpszName, bool isDeclaration);

public:
  // You must create your first instance of XMLNode with these 3 parse functions:
  // (see complete explanation of parameters below)

  static XMLNode createRoot(const TCHAR *lpszName);

  static XMLNode *parseString(const TCHAR *lpszXML, XMLResults *pResults=NULL);
  static XMLNode *parseFile(const char *lpszXML, XMLResults *pResults=NULL);
  static XMLNode *openFileHelper(const char *lpszXML);

  // The tag parameter should be the name of the first tag inside the XML file.
  // If the tag parameter is omitted, the 3 functions return a node that represents
  // the head of the xml document including the declaration term (<? ... ?>).

  // If the XML document is corrupted:
  //   - The "openFileHelper" method will stop execution and display an error message.
  //   - The 2 other methods will initialize the "pResults" variable with some information that
  //     can be used to trace the error.
  // you can have a detailed explanation of the parsing error with this function:

  static bool GlobalError;

  /**
   * Parse XML errors into a user friendly string.
   */
  static const TCHAR *getError(XMLError error);

  /**
   * name of the node
   */
  const TCHAR *getName() const {
    assert(d != NULL);

    return d->lpszName;
  }

  typedef XMLNodeData::const_iterator const_iterator;

  const_iterator begin() const {
    return d->begin();
  }

  const_iterator end() const {
    return d->end();
  }

  /**
   * @return the first child node, or NULL if there is none
   */
  gcc_pure
  const XMLNode *GetFirstChild() const {
    return d != NULL && !d->pChild.empty()
      ? &d->pChild.front()
      : NULL;
  }

  /**
   * @return ith child node with specific name (return an empty node
   * if failing)
   */
  gcc_pure
  const XMLNode *getChildNode(const TCHAR *name) const;

  /**
   * @return ith attribute content with specific name (return a NULL
   * if failing)
   */
  const TCHAR *getAttribute(const TCHAR *name) const;

  /**
   * Create an XML file from the head element.
   *
   * @param writer the stream to write the XML text to

   * @param nFormat 0 if no formatting is required, otherwise nonzero
   * for formatted text with carriage returns and indentation.
   */
  void serialise(TextWriter &writer, int nFormat) const;

  gcc_pure
  bool isDeclaration() const {
    assert(d != NULL);

    return d->isDeclaration;
  }

  // to allow shallow copy:
  ~XMLNode();

  /**
   * Shallow copy.
   */
  XMLNode(const XMLNode &A);

  XMLNode(XMLNode &&other)
    :d(other.d) {
    other.d = NULL;
  }

  /**
   * Shallow copy.
   */
  XMLNode& operator=(const XMLNode& A);

  XMLNode &operator=(XMLNode &&other) {
    std::swap(d, other.d);
    return *this;
  }

  static void destroyCurrentBuffer(XMLNodeData *d);

  gcc_constexpr_ctor
  XMLNode(): d(NULL) {}

  // The strings given as parameters for these 4 methods will be free'd by the XMLNode class:

  /**
   * Add a child node to the given element.
   */
  XMLNode &AddChild(const TCHAR *lpszName, bool isDeclaration);

  /**
   * Add an attribute to an element.
   */
  void AddAttribute(const TCHAR *lpszName, const TCHAR *lpszValuev);

  /**
   * Add text to the element.
   */
  void AddText(const TCHAR *lpszValue);

  void AddText(const TCHAR *text, size_t length);

private:
  // these are functions used internally (don't bother about them):
  bool ParseXMLElement(XML *pXML);

  /**
   * Creates an user friendly XML string from a given element with
   * appropriate white space and carriage returns.
   *
   * This recurses through all subnodes then adds contents of the
   * nodes to the string.
   */
  static void serialiseR(const XMLNodeData *pEntry, TextWriter &writer,
                         int nFormat);
};

#endif
