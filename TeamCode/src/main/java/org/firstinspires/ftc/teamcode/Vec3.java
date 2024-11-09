package org.firstinspires.ftc.teamcode;

public class Vec3 {

    public double x;
    public double y;
    public double z;

    public Vec3() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }

    public Vec3(double value) {
        x = value;
        y = value;
        z = value;
    }

    public Vec3(double X, double Y, double Z) {
        x = X;
        y = Y;
        z = Z;
    }

    public double product() {
        return x * y * z;
    }

    public String toString() {
        return "[" + x + ", " + y + ", " + z + "]";
    }

}
