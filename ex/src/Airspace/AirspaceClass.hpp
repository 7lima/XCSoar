#ifndef AIRSPACE_CLASS_HPP
#define AIRSPACE_CLASS_HPP

#include "Util/tstring.hpp"

enum AirspaceClass_t {
  OTHER= 0,
  RESTRICT,
  PROHIBITED,
  DANGER,
  CLASSA,
  CLASSB,
  CLASSC,
  CLASSD,
  NOGLIDER,
  CTR,
  WAVE,
  AATASK,
  CLASSE,
  CLASSF,
  AIRSPACECLASSCOUNT
};

const tstring airspace_class_as_text(const AirspaceClass_t item, 
                                     const bool consise=false);

#endif
