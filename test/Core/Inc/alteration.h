/*
 * alteration.h
 *
 *  Created on: Nov 6, 2021
 *      Author: Haeun Park
 */

#ifndef INC_ALTERATION_H_
#define INC_ALTERATION_H_



uint32_t map(uint32_t in, uint32_t inMin, uint32_t inMax, uint32_t outMin, uint32_t outMax) {
  // check it's within the range
  if (inMin < inMax) {
    if (in <= inMin)
      return outMin;
    if (in >= inMax)
      return outMax;
  } else { // Cope with input range being backwards.
    if (in >= inMin)
      return outMin;
    if (in <= inMax)
      return outMax;
  }
  // calculate how far into the range we are
  float scale = (float)((in - inMin) / (inMax - inMin));
  // calculate the output.
  return (uint32_t)(outMin + scale * (outMax - outMin));
}

#endif /* INC_ALTERATION_H_ */
