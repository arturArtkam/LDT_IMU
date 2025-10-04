#include "linalg.h"

float sclr_mltp(struct Vec a, struct Vec b)
{
    return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
}

struct Vec vctr_summ(struct Vec a, struct Vec b)
{
    struct Vec result;
    result.X = a.X + b.X;
    result.Y = a.Y + b.Y;
    result.Z = a.Z + b.Z;
    return result;
}

struct Vec vctr_diff(struct Vec a, struct Vec b)
{
    struct Vec result;
    result.X = a.X - b.X;
    result.Y = a.Y - b.Y;
    result.Z = a.Z - b.Z;
    return result;
}

struct Vec vctr_mltp(struct Vec a,  struct Vec b)
{
    struct Vec result;
    result.X = a.Y * b.Z - a.Z * b.Y;
    result.Y = a.Z * b.X - a.X * b.Z;
    result.Z = a.X * b.Y - a.Y * b.X;
    return result;
}

struct Vec vctr_mltp_n(float a, struct Vec b)
{
    struct Vec result;
    result.X = a * b.X;
    result.Y = a * b.Y;
    result.Z = a * b.Z;
    return result;
}

float modul(struct Vec a)
{
    return sqrtf(a.X * a.X + a.Y * a.Y + a.Z * a.Z);
}

float vctr_cos(struct Vec a, struct Vec b)
{
    return sclr_mltp(a, b) / (modul(a)*modul(b));

}

struct Vec mtrx_vctr_mltp(struct Matrix_3_3 m, struct Vec v)
{
    struct Vec result;
    result.X = v.X*m.XX + v.Y *m.YX + v.Z *m.ZX;
    result.Y = v.X*m.XY + v.Y *m.YY + v.Z *m.ZY;
    result.Z = v.X*m.XZ + v.Y *m.YZ + v.Z *m.ZZ;
    return result;
}

struct Matrix_3_3 mtrx_1(struct Matrix_3_3 m)
{
struct Matrix_3_3 m_1;
    float det = m.XX * (m.YY * m.ZZ - m.ZY * m.YZ) -
        m.XY * (m.YX * m.ZZ - m.YZ * m.ZX) +
        m.XZ * (m.YX * m.ZY - m.YY * m.ZX);
    float det_1 = 1 / det;
    m_1.XX = (m.YY*m.ZZ - m.ZY*m.YZ) * det_1;
    m_1.XY = (m.XZ*m.ZY - m.XY*m.ZZ) * det_1;
    m_1.XZ = (m.XY*m.YZ - m.XZ*m.YY) * det_1;
    m_1.YX = (m.YZ*m.ZX - m.YX*m.ZZ) * det_1;
    m_1.YY = (m.XX*m.ZZ - m.XZ*m.ZX) * det_1;
    m_1.YZ = (m.YX*m.XZ - m.XX*m.YZ) * det_1;
    m_1.ZX = (m.YX*m.ZY - m.ZX*m.YY) * det_1;
    m_1.ZY = (m.ZX* m.XY - m.XX*m.ZY) * det_1;
    m_1.ZZ = (m.XX* m.YY - m.YX*m.XY) * det_1;
    return m_1;
}

struct Matrix_3_3 mtrx_mltp(struct Matrix_3_3 m, struct Matrix_3_3 m1)
{
    struct Matrix_3_3 result;
    result.XX = m.XX*m1.XX + m.YX*m1.XY + m.ZX*m1.XZ;
    result.XY = m.XY*m1.XX + m.YY*m1.XY + m.ZY*m1.XZ;
    result.XZ = m.XZ*m1.XX + m.YZ*m1.XY + m.ZZ*m1.XZ;

    result.YX = m.XX*m1.YX + m.YX*m1.YY + m.ZX*m1.YZ;
    result.YY = m.XY*m1.YX + m.YY*m1.YY + m.ZY*m1.YZ;
    result.YZ = m.XZ*m1.YX + m.YZ*m1.YY + m.ZZ*m1.YZ;

    result.ZX = m.XX*m1.ZX + m.YX*m1.ZY + m.ZX*m1.ZZ;
    result.ZY = m.XY*m1.ZX + m.YY*m1.ZY + m.ZY*m1.ZZ;
    result.ZZ = m.XZ*m1.ZX + m.YZ*m1.ZY + m.ZZ*m1.ZZ;
    return result;
}

struct Matrix_3_3 mtrx_dv(struct Matrix_3_3 m, struct Matrix_3_3 m1)
{
    return mtrx_mltp( m, (mtrx_1(m1)));
}
