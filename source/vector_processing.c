#include "vector_processing.h"

void normalize_vector(VectorType *v) {
    if (v->m_y0 < v->m_y1) {
        uint8_t tmp_x = v->m_x0;
        uint8_t tmp_y = v->m_y0;
        v->m_x0 = v->m_x1;
        v->m_y0 = v->m_y1;
        v->m_x1 = tmp_x;
        v->m_y1 = tmp_y;
    }
}

int is_vector_invalid(VectorType *v) {
    float dx = (float)v->m_x1 - v->m_x0;
    float dy = (float)v->m_y1 - v->m_y0;
    float length = sqrtf(dx * dx + dy * dy);

    if (length < 1.0f)
        return 1;
    if (v->m_x0 >= 128 || v->m_y0 >= 128 || v->m_x1 >= 128 || v->m_y1 >= 128)
        return 1;
    if (v->m_x0 == v->m_x1 && v->m_y0 == v->m_y1)
        return 1;
    return 0;
}

void mark_vector_invalid(VectorType *v) {
    v->m_x0 = 128;
    v->m_y0 = 128;
    v->m_x1 = 128;
    v->m_y1 = 128;
    v->m_index = 255;
    v->m_flags = 255;
}

bool are_connected(VectorType *a, VectorType *b) {
    float dx1 = fabsf((float)a->m_x1 - b->m_x0);
    float dy1 = fabsf((float)a->m_y1 - b->m_y0);
    float dx2 = fabsf((float)b->m_x1 - a->m_x0);
    float dy2 = fabsf((float)b->m_y1 - a->m_y0);
    return ((dx1 <= 1 && dy1 <= 1) || (dx2 <= 1 && dy2 <= 1));
}

VectorType merge(VectorType *a, VectorType *b) {
    VectorType out;
    float dx = fabsf((float)a->m_x1 - b->m_x0);
    float dy = fabsf((float)a->m_y1 - b->m_y0);
    if (dx < 1 && dy < 1) {
        out.m_x0 = a->m_x0;
        out.m_y0 = a->m_y0;
        out.m_x1 = b->m_x1;
        out.m_y1 = b->m_y1;
    } else {
        out.m_x0 = b->m_x0;
        out.m_y0 = b->m_y0;
        out.m_x1 = a->m_x1;
        out.m_y1 = a->m_y1;
    }
    out.m_index = a->m_index;
    out.m_flags = a->m_flags;

    return out;
}

uint8_t merge_connected_vectors(VectorType *input, uint8_t count, VectorType *output) {
    bool used[8] = {false};
    uint8_t outCount = 0;

    for (uint8_t i = 0; i < count; i++) {
        if (used[i])
            continue;

        VectorType current = input[i];
        used[i] = true;
        bool extended;
        do {
            extended = false;
            for (uint8_t j = 0; j < count; j++) {
                if (i != j && !used[j] && are_connected(&current, &input[j])) {
                    current = merge(&current, &input[j]);
                    used[j] = true;
                    extended = true;
                }
            }
        } while (extended);

        output[outCount++] = current;
    }
    return outCount;
}

void PreprocessVectors(VectorType *inputVectors, uint8_t inputCount, VectorType *leftLine,
                       VectorType *rightLine) {
    VectorType valid[8];
    VectorType merged[8];
    uint8_t validCount = 0;

    for (uint8_t i = 0; i < inputCount; i++) {
        normalize_vector(&inputVectors[i]);
        if (!is_vector_invalid(&inputVectors[i])) {
            valid[validCount++] = inputVectors[i];
        }
    }
    if (validCount == 0) {
        mark_vector_invalid(leftLine);
        mark_vector_invalid(rightLine);
        stop = 1;
        return;
    }
    uint8_t mergedCount = merge_connected_vectors(valid, validCount, merged);

    int leftIdx = -1, rightIdx = -1;
    float minLeftDist = 9999.0f, minRightDist = 9999.0f;

    for (uint8_t i = 0; i < mergedCount; i++) {
        float centerX = (merged[i].m_x0 + merged[i].m_x1) / 2.0f;
        float distToCenter = fabsf(centerX - CENTER_X);
        float slope = ((float)merged[i].m_x1 - merged[i].m_x0) /
                      ((float)merged[i].m_y1 - merged[i].m_y0 + 0.01f);
        if (slope < 0) {
            if (distToCenter < minLeftDist) {
                minLeftDist = distToCenter;
                leftIdx = i;
            }
        } else {
            if (distToCenter < minRightDist) {
                minRightDist = distToCenter;
                rightIdx = i;
            }
        }
    }

    if (leftIdx != -1) {
        *leftLine = merged[leftIdx];
    } else {

        leftLine->m_x0 = 0;
        leftLine->m_y0 = 51;
        leftLine->m_x1 = 0;
        leftLine->m_y1 = 0;
        leftLine->m_index = 255;
        leftLine->m_flags = 255;
    }
    if (rightIdx != -1) {
        *rightLine = merged[rightIdx];
    } else {
        rightLine->m_x0 = 78;
        rightLine->m_y0 = 51;
        rightLine->m_x1 = 78;
        rightLine->m_y1 = 0;
        rightLine->m_index = 255;
        rightLine->m_flags = 255;
    }

    if (is_vector_invalid(leftLine) && is_vector_invalid(rightLine)) {
        stop = 1;
    } else {
        stop = 0;
    }
}
