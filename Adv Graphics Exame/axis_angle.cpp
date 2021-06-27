#include <math.h>

#include "axis_angle.h"

AxisAngle AxisAngle::from(const Quaternion& q)
{
    if (q.w > 1.0)
        normalized(q);

    AxisAngle ax;
    ax.angle = 2 * acos(q.w);
    Scalar k = sqrt(1.0 - q.w * q.w);
    if (k < EPSILON)
    {
        ax.axis.x = q.x;
        ax.axis.y = q.y;
        ax.axis.z = q.z;
    }
    else
    {
        ax.axis.x = q.x / k;
        ax.axis.y = q.y / k;
        ax.axis.z = q.z / k;
    }
    return ax;
}