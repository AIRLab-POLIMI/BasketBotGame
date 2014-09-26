/***************************************************************************
                          point.h  -  description
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



#ifndef point_h
#define point_h 1

#ifdef DMALLOC

#include <dmalloc.h>

#endif

/**
* It implement the father father class of point. It is the representation a single point.
* @short Class implementing a point.
*/

class point
{
  public:
  /**
   * Constructor generated by the UML software used to build Brian. It return an empty point.
   */
  point();

  /**
   * Copy constructor generated by the UML software used to build Brian. It return a copy of the point.
   */
  point(const point &right);

  /*
   * Constructor that build a point with coordinates xa, yb.
   * @param axa is the abscissa of the point
   * @param aya is the ordinate of the point
   */
  point(float axa, float aya);

  /**	
   * Destructor destroy a point
   */
  ~point();

  /**
   * Return the abscissa of the point
   * @return a value
   */
  const float get_xa ();
  /**
   * Set the abscissa of the point
   * @param value the new value for the abscissa
   */
  void set_xa (float value);

  /**
   * Return the ordinate of the point
   * @return a value
   */
  const float get_ya ();
  /**
   * Set the ordinate of the point
   * @param value the new value for the ordinate
   */
  void set_ya (float value);

 private: 
    // Data Members for Class Attributes

  float xa;
  
  float ya;

};

// Class figure_point 

inline const float point::get_xa ()
{
  return xa;
}

inline void point::set_xa (float value)
{
  xa = value;
}

inline const float point::get_ya ()
{
  return ya;
}

inline void point::set_ya (float value)
{
  ya = value;
}

#endif