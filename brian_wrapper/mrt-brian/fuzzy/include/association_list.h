/***************************************************************************
                          association_list.h  -  description
                             -------------------
    begin                : Wed Sep 13 2000
    copyright            : (C) 2000 by Halva Giovanni & Giacomo
    email                : invehalv@airlab.elet.polimi.it
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


#ifndef association_list_h
#define association_list_h 1

#ifdef DMALLOC

#include <dmalloc.h>

#endif

#include "association_set_multimap.h"


/**
* It is a multimap that collect the associations between crisp and fuzzy data
* @short Collection of associations
*/

class association_list : public association_set_multimap  //## Inherits: association_list inheritance%3835139F0064
{
  public:
  /**
   * Constructor generated by the UML software used to build Brian. It return an empty association list.
   */
  association_list();

  /**
   * Copy constructor generated by the UML software used to build Brian. It return a copy of the association list.
   */
  association_list(const association_list &right);

  /**	
   * Destructor destroy an association list freeing all the association self-contained.
   */
  ~association_list();


  /**
   * Method that given the label of the association return a pointer to	the association who have the corrispondent label. It point to the original association so it must not be destroyed.
   * @param alabel is the label of the data
   * @return a pointer to an association
   */ 
  association* get_association ( const char* alabel);

   /**
    * Add an assocition to the list.
    * @param aassoc is the pointer to the association to add
    */
  void add_association (association *aassoc);
};


#endif
