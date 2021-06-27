#include <math.h>

#include "matrix3.h"

Matrix3::Matrix3() : x(1, 0, 0), y(0, 1, 0), z(0, 0, 1) {}

Matrix3::Matrix3(Scalar m00, Scalar m01, Scalar m02,
    Scalar m10, Scalar m11, Scalar m12,
    Scalar m20, Scalar m21, Scalar m22) : x(m00, m10, m20), y(m01, m11, m21), z(m02, m12, m22) {}

// combine two rotations (r goes first!)
Matrix3 Matrix3::operator*(Matrix3 r) const
{
    Matrix3 MatComb = Matrix3(
        r.x.x * x.x + r.y.x * x.y + r.z.x * x.z, r.x.x * y.x + r.y.x * y.y + r.z.x * y.z, r.x.x * z.x + r.y.x * z.y + r.z.x * z.z,
        r.x.y * x.x + r.y.y * x.y + r.z.y * x.z, r.x.y * y.x + r.y.y * y.y + r.z.y * y.z, r.x.y * z.x + r.y.y * z.y + r.z.y * z.z,
        r.x.z * x.z + r.y.z * x.y + r.z.z * x.z, r.x.z * y.z + r.y.z * y.y + r.z.z * y.z, r.x.z * z.z + r.y.z * z.y + r.z.z * z.z);
    return MatComb;
}

Matrix3 Matrix3::operator*(Scalar b) const
{
    return Matrix3(
        x.x * b, y.x * b, z.x * b,
        x.y * b, y.y * b, z.y * b,
        x.z * b, y.z * b, z.z * b);
}

Matrix3 Matrix3::operator-(Matrix3 r) const
{
    return Matrix3(
        x.x - r.x.x, y.x - r.y.x, z.x - r.z.x,
        x.y - r.x.y, y.y - r.y.y, z.y - r.z.y,
        x.z - r.x.z, y.z - r.y.z, z.z - r.z.z
    );
}

Matrix3 Matrix3::operator+(Matrix3 r) const
{
    return Matrix3(
        x.x + r.x.x, y.x + r.y.x, z.x + r.z.x,
        x.y + r.x.y, y.y + r.y.y, z.y + r.z.y,
        x.z + r.x.z, y.z + r.y.z, z.z + r.z.z
    );
}

Vector3 Matrix3::apply(Vector3 v) const 
{
    return Vector3(x.x * v.x + y.x * v.y + z.x * v.z,
        x.y * v.x + y.y * v.y + z.y * v.y,
        x.z * v.x + y.z * v.y + z.z * v.z);
}

Versor3 Matrix3::axisX() const
{
    return normalize(x);
}

Versor3 Matrix3::axisY() const
{
    return normalize(y);
}

Versor3 Matrix3::axisZ() const
{
    return normalize(z);
}

Matrix3 Matrix3::inverse(Matrix3 m) const
{
    Matrix3 t;
    t.x.x = m.z.z * m.y.y - m.y.z * m.z.y;
    t.y.x = m.y.z * m.z.x - m.z.z * m.y.x;
    t.z.x = m.z.y * m.y.x - m.y.y * m.z.x;
    t.x.y = m.x.z * m.z.y - m.z.z * m.x.y;
    t.y.y = m.z.z * m.x.x - m.x.z * m.z.x;
    t.z.y = m.x.y * m.z.x - m.z.y * m.x.x;
    t.x.z = m.y.z * m.x.y - m.x.z * m.y.y;
    t.y.z = m.x.z * m.y.x - m.y.z * m.x.x;
    t.z.z = m.y.y * m.x.x - m.x.y * m.y.x;
    return t * (1 / determinate(m));
}

void Matrix3::invert() const
{
    inverse(*this);
}

Matrix3 Matrix3::lookAt(Point3 eye, Point3 target, Versor3 up = Versor3::up()) 
{
    Versor3 p = normalize(target - eye);
    Matrix3 mat;
    mat.z.asVersor() = p;
    mat.x.asVersor() = normalize(cross(up.asVector(), mat.z));
    mat.y.asVersor() = normalize(cross(mat.z, mat.x));
    return mat;
}

// returns a rotation
Matrix3 Matrix3::toFrom(Versor3 to, Versor3 from) 
{
    Vector3 cr = cross(to, from);
    Scalar d = dot(to, from);
    Scalar k = 1.0 / (1.0 + d);

    Matrix3 matFrom(
        (cr.x * cr.x * k) + d, (cr.y * cr.x * k) - cr.z, (cr.z * cr.x * k) + cr.y,
        (cr.x * cr.y * k) + cr.z, (cr.y * cr.y * k) + d, (cr.z * cr.y * k) - cr.x,
        (cr.x * cr.z * k) - cr.y, (cr.y * cr.z * k) + cr.x, (cr.z * cr.z * k) + d);
    return matFrom;
}

Matrix3 Matrix3::toFrom(Vector3 to, Vector3 from) 
{
    return toFrom(normalize(to), normalize(from));
}

Matrix3 Matrix3::from(const Quaternion& m)
{
    Scalar X = m.x;
    Scalar Y = m.y;
    Scalar Z = m.z;
    Scalar W = m.w;

    Matrix3 mat = Matrix3(
        1 - 2 * (Y * Y) - 2 * (Z * Z), 2 * (X * Y) + 2 * (W * Z), 2 * (X * Z) - 2 * (W * Y),
        2 * (X * Y) - 2 * (W * Z), 1 - 2 * (X * X) - 2 * (Z * Z), 2 * (Y * Z) + 2 * (W * X),
        2 * (X * Z) + 2 * (W * Y), 2 * (Y * Z) - 2 * (W * X), 1 - 2 * (X * X) - 2 * (Y * Y)
    );
    return mat;
}

Matrix3 Matrix3::from(const Euler& e)
{
    Matrix3 matRZ = rotationZ(e.roll);
    Matrix3 matRY = rotationY(e.yaw);
    Matrix3 matRX = rotationX(e.pitch);
    Matrix3 result = ((matRZ * matRX) * matRY);
    return result;
}

Matrix3 Matrix3::from(const AxisAngle& e)
{
    Matrix3 matrix;
    Scalar m_cos = cos(e.angle);
    Scalar m_sin = sin(e.angle);
    Scalar offset = 1.0 - m_cos;

    matrix.x.x = m_cos + (e.axis.x * e.axis.x * offset);
    matrix.y.y = m_cos + (e.axis.y * e.axis.y * offset);
    matrix.z.z = m_cos + (e.axis.z * e.axis.z * offset);

    Scalar aux_1 = e.axis.x * e.axis.y * offset;
    Scalar aux_2 = e.axis.z * m_sin;
    matrix.x.y = aux_1 + aux_2;
    matrix.y.x = aux_1 - aux_2;

    aux_1 = e.axis.x * e.axis.z * offset;
    aux_2 = e.axis.y * m_sin;
    matrix.x.z = aux_1 - aux_2;
    matrix.z.x = aux_1 + aux_2;

    aux_1 = e.axis.y * e.axis.z * offset;
    aux_2 = e.axis.x * m_sin;
    matrix.y.z = aux_1 + aux_2;
    matrix.z.y = aux_1 - aux_2;

    return matrix;
}

bool Matrix3::isRot() const
{
    bool isARotationMatrix = false;

    if (determinate() == 1.0 && cross(x, y).isEqual(z) && cross(x, z).isEqual(y) && cross(y, z).isEqual(x))
        isARotationMatrix = true;

    return isARotationMatrix;
}

Matrix3 Matrix3::rotationX(Scalar angleDeg)
{
    Matrix3 XRot;
    XRot.y.y = cos(angleDeg);
    XRot.y.z = sin(angleDeg);
    XRot.z.y = -sin(angleDeg);
    XRot.z.z = cos(angleDeg);

    return XRot;
}

Matrix3 Matrix3::rotationY(Scalar angleDeg)
{
    Matrix3 YRot;
    YRot.x.x = cos(angleDeg);
    YRot.x.z = -sin(angleDeg);
    YRot.z.x = sin(angleDeg);
    YRot.z.z = cos(angleDeg);

    return YRot;
}

Matrix3 Matrix3::rotationZ(Scalar angleDeg)
{
    Matrix3 ZRot;
    ZRot.x.x = cos(angleDeg);
    ZRot.y.x = -sin(angleDeg);
    ZRot.x.y = sin(angleDeg);
    ZRot.y.y = cos(angleDeg);

    return ZRot;
}

void Matrix3::printf() const
{
    std::cout << x.x << y.x << z.x << "\n"
        << x.y << y.y << z.y << "\n"
        << x.z << y.z << z.z << std::endl;
}

Scalar Matrix3::determinate() const
{
    Scalar k1 = x.x * (z.z * y.y - y.z * z.y);
    Scalar k2 = x.y * (z.z * y.x - y.z * z.x);
    Scalar k3 = x.z * (z.y * y.x - y.y * z.x);
    return k1 - k2 + k3;
}

Scalar Matrix3::determinate(const Matrix3& m) const
{
    Scalar k1 = m.x.x * (m.z.z * m.y.y - m.y.z * m.z.y);
    Scalar k2 = m.x.y * (m.z.z * m.y.x - m.y.z * m.z.x);
    Scalar k3 = m.x.z * (m.z.y * m.y.x - m.y.y * m.z.x);
    return k1 - k2 + k3;
}