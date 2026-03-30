#ifndef VECTOR_PROCESSING_H
#define VECTOR_PROCESSING_H

#include "common_types.h"
#include "config.h"
#include "fsl_debug_console.h"
#include "fsl_uart.h"
#include "globals.h"
#include <math.h>
#include <stdbool.h>

// Preprocesează vectorii detectați de cameră și extrage 2 linii principale
void PreprocessVectors(VectorType *inputVectors, uint8_t inputCount, VectorType *leftLine,
                       VectorType *rightLine);
int is_vector_invalid(VectorType *v);
void mark_vector_invalid(VectorType *v);
uint8_t merge_connected_vectors(VectorType *input, uint8_t count, VectorType *output);
void normalize_vector(VectorType *v);

#endif
