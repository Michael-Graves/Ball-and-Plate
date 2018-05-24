/*
 * touchCalibration.c
 *
 *  Created on: Mar 30, 2018
 *      Author: Michael Graves
 *
 *  Inverse Bilinear interpolation adapted from: iquilezles.org/www/articles/ibilinear/ibilinear.htm
 */

/*
 * NOTE THIS ISN'T USED BUT COULD BE USED IF THE TOUCH SCREEN ISN'T GOOD
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "driverlib/fpu.h"
#include "touchCalibration.h"

#define CALIBRATION_NUM_X 5
#define CALIBRATION_NUM_Y 3
#define CALIBRATION_NUM (CALIBRATION_NUM_X * CALIBRATION_NUM_Y)

#define SCALE 4096

typedef struct vec2 vec2;

// Lookup tables to convert from screen position (S(x,y)) to touchscreen positions (index of these arrays in cm)
const uint16_t calibrationSX[CALIBRATION_NUM] = {0,  1000,  2000,  3000,  4000,
                                                 0,  1000,  2000,  3000,  4000,
                                                 0,  1000,  2000,  3000,  4000
};

const uint16_t calibrationSY[CALIBRATION_NUM] = {0,     0,     0,     0,     0,
                                                 1000,  1000,  1000,  1000,  1000,
                                                 2000,  2000,  2000,  2000,  2000
};

struct vec2 {
    float x, y;
};

vec2 mkVec2(float x, float y) {
    vec2 out = { x, y };
    return out;
}

vec2 add(vec2 a, vec2 b) {
    return mkVec2(a.x + b.x, a.y + b.y);
}

vec2 sub(vec2 a, vec2 b) {
    return mkVec2(a.x - b.x, a.y - b.y);
}

float cross(vec2 a, vec2 b) {
    return (a.x * b.y) - (a.y * b.x);
}

vec2 invBilinear(vec2 p, vec2 a, vec2 b, vec2 c, vec2 d) {
    vec2 e = sub(b, a);                     // b-a
    vec2 f = sub(d, a);                     // d-a
    vec2 g = add(sub(a,b),sub(c,d));        // a-b+c-d
    vec2 h = sub(p, a);

    float k2 = cross(g, f);
    float k1 = cross(e, f) + cross(h, g);
    float k0 = cross(h, e);

    float w = k1 * k1 - 4.0f * k0 * k2;

    if(w < 0.0f) {
        return mkVec2(-1.0f, -1.0f);
    }
    w = sqrt(w);

    float v1 = (-k1 - w)/(2.0f * k2);
    float u1 = (h.x - (f.x * v1))/(e.x + (g.x * v1));

    float v2 = (-k1 + w)/(2.0f * k2);
    float u2 = (h.x - (f.x * v2))/(e.x + (g.x * v2));

    float u = u1;
    float v = v1;

    if( v < 0.0f || v > 1.0f || u < 0.0f || u > 1.0f ) {
        u = u2;
        v = v2;
    }

    if( v < 0.0f || v > 1.0f || u < 0.0f || u > 1.0f ) {
        u = -1.0f;
        v = -1.0f;
    }

    return mkVec2(u, v);
}

vec2 GetS(uint8_t x, uint8_t y) {
    uint8_t index = x + (y * CALIBRATION_NUM_X);
    return mkVec2(calibrationSX[index], calibrationSY[index]);
}

void GetCM(uint16_t sx, uint16_t sy, uint8_t x, uint8_t y, uint32_t *xout, uint32_t *yout) {
    vec2 output = invBilinear(mkVec2(sx, sy), GetS(x, y), GetS(x+1, y), GetS(x+1, y+1), GetS(x, y+1));

    *xout = x * SCALE + (uint16_t)(output.x * SCALE);
    *yout = y * SCALE + (uint16_t)(output.y * SCALE);
}

uint16_t GetSX(uint8_t x, uint8_t y) {
    return calibrationSX[x + (y * CALIBRATION_NUM_X)];
}

uint16_t GetSY(uint8_t x, uint8_t y) {
    return calibrationSY[x + (y * CALIBRATION_NUM_X)];
}

uint16_t Max(uint16_t a, uint16_t b) {
    if(a > b) return a;
    return b;
}

uint16_t Min(uint16_t a, uint16_t b) {
    if(a < b) return a;
    return b;
}

void GetBoundingBox(uint8_t x, uint8_t y, uint16_t *minX, uint16_t *minY, uint32_t *maxX, uint32_t *maxY) {
    *minX = Min(GetSX(x, y), GetSX(x, y+1));
    *minY = Min(GetSY(x, y), GetSY(x+1, y));

    *maxX = Max(GetSX(x+1, y), GetSX(x+1, y+1));
    *maxY = Max(GetSY(x, y+1), GetSY(x+1, y+1));
}

_Bool InsidePolygon(uint16_t sx, uint16_t sy, uint8_t x, uint8_t y) {
    // Fast check (See if point is within bounding box of region), Could be optimized
    uint16_t minX, minY, maxX, maxY;
    GetBoundingBox(x, y, &minX, &minY, &maxX, &maxY);
    if(sx >= minX && sx <= maxX && sy >= minY && sy <= maxY) {
        uint8_t count = 0;
        //Line 1: (x,y) to (x,y+1)
        uint16_t x1 = GetSX(x, y);
        uint16_t y1 = GetSY(x, y);
        uint32_t intersectx = ((uint32_t)(sy - y1) * (GetSX(x,y+1) - x1))/(GetSY(x,y+1) - y1) + x1; // x = (y-b)/m + a
        if(intersectx > sx) count++;

        //Line 2: (x,y+1) to (x+1,y+1)
        x1 = GetSX(x, y+1);
        y1 = GetSY(x, y+1);
        intersectx = ((uint32_t)(sy - y1) * (GetSX(x+1,y+1) - x1))/(GetSY(x+1,y+1) - y1) + x1; // x = (y-b)/m + a
        if(intersectx > sx) count++;

        //Line 3: (x+1,y) to (x+1,y+1)
        x1 = GetSX(x+1, y);
        y1 = GetSY(x+1, y);
        intersectx = ((uint32_t)(sy - y1) * (GetSX(x+1,y+1) - x1))/(GetSY(x+1,y+1) - y1) + x1; // x = (y-b)/m + a
        if(intersectx > sx) count++;

        //Line 4: (x,y+1) to (x+1,y+1)
        x1 = GetSX(x, y);
        y1 = GetSY(x, y);
        intersectx = ((uint32_t)(sy - y1) * (GetSX(x+1,y) - x1))/(GetSY(x+1,y) - y1) + x1; // x = (y-b)/m + a
        if(intersectx > sx) count++;

        //Check if count is odd (inside polygon)
        if((count % 2) == 1) {
            return true;
        }
    }
    return false;
}

void GetPosition(uint32_t sx, uint32_t sy, uint32_t *xout, uint32_t *yout) {
    // Find which region contains s(x,y)
    uint8_t x, y;
    for (x = 0; x < (CALIBRATION_NUM_X - 1); y++) {
        for (y = 0; y < (CALIBRATION_NUM_Y - 1); y++) {
            if(InsidePolygon(sx, sy, x, y)) {
                GetCM(sx, sy, x, y, &xout, &yout);
                return;
            }
        }
    }
}


