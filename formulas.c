#include "formulas.h"

int checkOverlap( int object1X, int object1width,
                    int object1Y, int object1height,
                    int object2X, int object2width,
                    int object2Y, int object2height)
{
    int xOverlap, yOverlap;
    int min1st, max1st, min2nd, max2nd;

    min1st = object1X;
    max1st = object1X+object1width;
    min2nd = object2X;
    max2nd = object2X+object2width;
    xOverlap = ((min2nd <= max1st) && (max1st <= max2nd))
                || ((min1st <= min2nd) && (min2nd <= max1st))
                || ((min2nd <= min1st) && (min1st <= max2nd))
                || ((min1st <= max2nd) && (max2nd <= max1st));


    min1st = object1Y;
    max1st = object1Y+object1height;
    min2nd = object2Y;
    max2nd = object2Y+object2height;
    yOverlap = ((min2nd <= max1st) && (max1st <= max2nd))
                || ((min1st <= min2nd) && (min2nd <= max1st))
                || ((min2nd <= min1st) && (min1st <= max2nd))
                || ((min1st <= max2nd) && (max2nd <= max1st));

    return (xOverlap && yOverlap);

}


