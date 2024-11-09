package org.firstinspires.ftc.teamcode;

public class NMath {

    //
    //** NORMAL MATH **//
    //
    public static float toRadians(float degrees) {
        return (float) (degrees * (Math.PI/180));
    }

    public static float clamp(float value, float min, float max) {
        if (value < min) {
            return min;
        } else if (value > max){
            return max;
        } else {
            return value;
        }
    }

    //
    //** VECTOR MATH **//
    //
    public static Vec3 Add(Vec3 vecA, Vec3 vecB) {
        return new Vec3(vecA.x + vecB.x, vecA.y + vecB.y, vecA.z + vecB.z);
    }

    public static Vec3 Subtract(Vec3 vecA, Vec3 vecB) {
        return new Vec3(vecA.x - vecB.x, vecA.y - vecB.y, vecA.z - vecB.z);
    }

    public static Vec3 Multiply(Vec3 vecA, Vec3 vecB) {
        return new Vec3(vecA.x * vecB.x, vecA.y * vecB.y, vecA.z * vecB.z);
    }

    public static Vec3 Multiply(Vec3 vecA, float factorToMulti) {
        return new Vec3(vecA.x * factorToMulti, vecA.y * factorToMulti, vecA.z * factorToMulti);
    }

    public static Vec3 Divide(Vec3 vecA, Vec3 vecB) {
        return new Vec3(vecA.x / vecB.x, vecA.y / vecB.y, vecA.z / vecB.z);
    }

    public static Vec3 Divide(Vec3 vecA, float factorToDiv) {
        return new Vec3(vecA.x / factorToDiv, vecA.y / factorToDiv, vecA.z / factorToDiv);
    }

    public static Vec3 Pow(Vec3 vecA, int pow) {
        Vec3 tempVec = vecA;
        for (int i = 0; i < pow; i++) {
            tempVec = Multiply(tempVec, vecA);
        }
        return tempVec;
    }

    public static float GetLength(Vec3 vecA) {
        return (float) Math.sqrt(vecA.x*vecA.x + vecA.y*vecA.y + vecA.z*vecA.z);
    }

    public static Vec3 Normalize(Vec3 vecA) {
        return Divide(vecA, GetLength(vecA)); // Divides itself by its magnitude
    }

    public static float DotProduct(Vec3 vecA, Vec3 vecB) {
        return (float) ((vecA.x * vecB.x) + (vecA.y * vecB.y) + (vecA.z *vecB.z));
    }

    public static Vec3 CrossProduct(Vec3 a, Vec3 b) {
        return new Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
    }

    public static float Distance(Vec3 vecA, Vec3 vecB) {
        return (float) Math.sqrt(Math.pow((vecA.x-vecB.x), 2) + Math.pow((vecA.y-vecB.y), 2) + Math.pow((vecA.z-vecB.z), 2));
    }

}
