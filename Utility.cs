using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Utility
{
    /// <summary>
    /// Returns whether the two lines intersect.
    /// </summary>
    /// <param name="a1">start point of line A</param>
    /// <param name="a2">end point of line A</param>
    /// <param name="b1">start point of line B</param>
    /// <param name="b2">end point of line B</param>
    /// <returns></returns>
    public static bool IsCross(Vector3 a1, Vector3 a2, Vector3 b1, Vector3 b2)
    {
        // get line from vector
        Vector3 lineA = a2 - a1;
        Vector3 lineB = b2 - b1;

        // calculate perpendicular vector not normalize
        Vector3 perpA = new(-lineA.y, lineA.x);
        Vector3 perpB = new(-lineB.y, lineB.x);

        // Dot product
        float dotA1 = Vector3.Dot(perpB, a1 - b1);
        float dotA2 = Vector3.Dot(perpB, a2 - b1);
        float dotB1 = Vector3.Dot(perpA, b1 - a1);
        float dotB2 = Vector3.Dot(perpA, b2 - a1);

        return dotA1 * dotA2 < 0 && dotB1 * dotB2 < 0;
    }

    /// <summary>
    /// Returns the vector on the line segment closest to the point.
    /// </summary>
    /// <param name="lineStart"></param>
    /// <param name="lineEnd"></param>
    /// <param name="point"></param>
    /// <param name="clamp"></param>
    /// <returns></returns>
    public static Vector3 GetClosestPointOnLine(Vector3 lineStart, Vector3 lineEnd, Vector3 point, bool clamp)
    {
        // get line from vector
        Vector3 toPoint = point - lineStart;
        Vector3 line = lineEnd - lineStart;

        // calculate where the nearest point
        float dot = Vector3.Dot(toPoint, line);
        float t = dot / line.sqrMagnitude;

        // clamp if you want
        t = clamp ? Mathf.Clamp01(t) : t;

        return (line * t) + lineStart;
    }

    /// <summary>
    /// Returns signed angle(ccw).
    /// </summary>
    /// <param name="from"></param>
    /// <param name="to"></param>
    /// <returns></returns>
    public static float GetSignedAngle(Vector3 from, Vector3 to)
    {
        // 179���� �������� ���� -179�� �ٲ� Ʀ
        float unsignedAngle = Vector3.Angle(from, to);
        float sign = Mathf.Sign((from.x * to.y) - (from.y * to.x));
        float signedAngle = unsignedAngle * sign;
        return signedAngle;
    }

    public static float GetSignedAngle(Vector3 from, Vector3 to, Vector3 pivot)
    {
        return GetSignedAngle(from - pivot, to - pivot);
    }

    public static Vector3 RotateVector(Vector3 origin, float angle)
    {
        float x = origin.x;
        float y = origin.y;
        float rad = angle * Mathf.Deg2Rad;
        float sin = Mathf.Sin(rad);
        float cos = Mathf.Cos(rad);
        Vector3 result = new((x * cos) - (y * sin), (x * sin) + (y * cos));
        return result;
    }

    public static Vector3 RotateVector(Vector3 origin, float angle, Vector3 pivot)
    {
        // only test
        Vector3 rotate = origin - pivot;
        Vector3 result = RotateVector(rotate, angle);
        return result + pivot;
    }

    static float Perpendicular(Vector3 lhs,  Vector3 rhs)
    {
        return (lhs.x * rhs.y) - (lhs.y * rhs.x);
    }

    public static Vector3 FindIntersection(Vector3 a1, Vector3 a2, Vector3 b1, Vector3 b2)
    {
        Vector3 line1 = a2 - a1;
        Vector3 line2 = b2 - b1;
        Vector3 line3 = b1 - a1;

        float denominator = Perpendicular(line1, line2);

        if (denominator == 0)
        {
            return Vector3.zero;
        }

        float numerator = Perpendicular(line3, line1);
        float t = numerator / denominator;
        Vector3 intersection = (line2 * t) + line3 + a1;

        return intersection;
    }

    public static bool IsPointOnRightSide(Vector3 a,  Vector3 b, Vector3 p)
    {
        Vector3 line = b - a;
        Vector3 point = p - a;

        float cross = (line.x * point.y) - (line.y * point.x);
        // ���ΰ� ��ġ�� �����ʿ��ִ°� �ƴ϶�� ������
        // �������̶�� ������ Z�� �������� ������� �ٶ󺸴� ����
        return cross < 0;
    }

    public static bool IsPointInTriangle(Vector3 t1, Vector3 t2, Vector3 t3,  Vector3 p)
    {
        bool check1, check2, check3;

        check1 = IsPointOnRightSide(t1, t2, p);
        check2 = IsPointOnRightSide(t2, t3, p);
        check3 = IsPointOnRightSide(t3, t1, p);

        return (check1 == check2) && (check2 == check3);
    }

    public static (Vector3, Vector3) CalculateBallReflection(Vector3 aVelocity, float aMass, Vector3 bVelocity, float bMass, float e)
    {
        Vector3 newAVelocity = (((aMass - (bMass * e)) * aVelocity) + ((1 + e) * bMass * bVelocity)) / (aMass + bMass);
        Vector3 newBVelocity = (((bMass - (aMass * e)) * bVelocity) + ((1 + e) * aMass * aVelocity)) / (aMass + bMass);

        return (newAVelocity, newBVelocity);
    }

    public static Vector3 Friction(Vector3 slopeNormal, Vector3 afterVelocity, Vector3 gravityDir, float gravity, float mass, float mu, float depth)
    {
        depth = 1 + depth;
        Vector3 slope = new Vector3(slopeNormal.y, -slopeNormal.x).normalized;
        float mg = mass * gravity;
        Vector3 mgV = mg * gravityDir;
        
        float theta = Vector3.Angle(Vector3.right, slope) * Mathf.Deg2Rad;
        float cosTheta = Mathf.Cos(theta);
        float sinTheta = Mathf.Sin(theta);

        float mgCosMag = cosTheta * mg;
        Vector3 mgCos = -1 * mgCosMag * slopeNormal;

        Vector3 fricDir = (mgCos - mgV).normalized;
        Vector3 fric = depth * mg * sinTheta * fricDir;
        Vector3 fricMax = depth * mgCosMag * mu * fricDir;

        bool fricCheck = fricMax.sqrMagnitude < fric.sqrMagnitude;
        Vector3 applyFric = fricCheck ? fricMax : fric;

        float dot = Vector3.Dot(applyFric, afterVelocity);
        if (dot > 0)
        {
            applyFric *= -1;
        }

        Vector3 projection = Vector3.Dot(slope, afterVelocity) * slope;
        if (projection.sqrMagnitude < applyFric.sqrMagnitude)
        {
            applyFric = -1 * projection;
        }

        return applyFric;
    }

    public static bool Approximately(float a, float b, float tolerance)
    {
        return Mathf.Abs(a - b) <= tolerance;
    }

    public static bool IsClockWise(Vector3[] vertices)
    {
        // if camera.z < 0 then result < 0 = ClockWise
        float result = 0;

        for (int i = 0; i < vertices.Length; i++)
        {
            int n = (i + 1) % vertices.Length;
            float cross = (vertices[i].x * vertices[n].y) - (vertices[i].y * vertices[n].x);
            result += cross;
        }

        return result < 0;
    }
}