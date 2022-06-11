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
// float q[3]  aktualna pozycja w przegubach
// const float t[3] - xyz to co chcemy osiagnac

void newPositions(Position *Pos, const float t[3])
{
    Pos->q1 = t[2] - 163; // mm
    float x, y, l, m;

    x = t[0];
    y = t[1];

    l = pow(x, 2) + pow(y, 2) - pow(a2, 2) - pow(a3, 2);
    m = 2 * a2 * a3;
    Pos->q2 = acos(l / m) * 180 / PI; // st.

    l = a3 * sin(Pos->q2);
    m = a2 + a3 * cos(Pos->q2);
    Pos->q3 = (atan(y / x) - atan(l / m)) * 180 / PI; // st.

    Pos->x=t[0];
    Pos->y=t[1];
    Pos->z=t[2];
}
