/*
 * geometrik.c
 *
 *  Created on: 27 maj 2022
 *      Author: maciej
 */

#include "geometrik.h"
#include <math.h>

#define a2 150
#define a3 15
#define PI 3.14159265
/*
int main()
{
    float q[3] = {0, 0, 0};
    float p[3][3] = {{0, 35, 176}, {150, 35, 188}, {165, 35, 163}};
    float t[3] = {0, 150, 183};

    newPositions(q, p, t);
}
*/
int newPositions(float q[3], const float t[3])
{
    q[0] = t[2] - 163; // mm
    float x, y, l, m;

    x = t[0];
    y = t[1];

    l = pow(x, 2) + pow(y, 2) - pow(a2, 2) - pow(a3, 2);
    m = 2 * a2 * a3;
    q[2] = acos(l / m) * 180 / PI; // st.

    l = a3 * sin(q[2]);
    m = a2 + a3 * cos(q[2]);
    q[1] = (atan(y / x) - atan(l / m)) * 180 / PI; // st.

    return 0;
}