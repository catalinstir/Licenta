#ifndef VECTOR_PROCESSING_H
#define VECTOR_PROCESSING_H

#include "common_types.h"
#include "config.h"
#include <math.h>
#include <stdbool.h>

/* Returns true if at least one valid lane line was found.
 * Returns false when both lines are invalid — caller should set stop = 1. */
bool PreprocessVectors(VectorType *inputVectors, uint8_t inputCount,
                       VectorType *leftLine, VectorType *rightLine);

int     is_vector_invalid(VectorType *v);
void    mark_vector_invalid(VectorType *v);
uint8_t merge_connected_vectors(VectorType *input, uint8_t count, VectorType *output);
void    normalize_vector(VectorType *v);

#endif
