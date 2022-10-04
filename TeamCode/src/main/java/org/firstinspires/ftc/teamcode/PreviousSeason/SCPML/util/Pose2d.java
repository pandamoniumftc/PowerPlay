package org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class Pose2d {

    public double X = 0;
    public double Y = 0;
    public double heading = 0;

    public Pose2d(double x, double y, double Heading) {
        X = x;
        Y = y;
        heading = Heading;
    }

    public Pose2d(double x, double y) {
        X = x;
        Y = y;
    }

    public Pose2d(double XAndY) {
        X = XAndY;
        Y = XAndY;
    }

    public Pose2d(Vector2d vector2d) {
        X = vector2d.X;
        Y = vector2d.Y;
    }

    public Pose2d(Vector2d vector2d, double heading) {
        X = vector2d.X;
        Y = vector2d.Y;
        this.heading = heading;
    }

    public Pose2d() {}

    public void add( Pose2d add ) {
        X += add.X;
        Y += add.Y;
    }

    public void sub( Pose2d subtract ) {
        X += subtract.X;
        Y -= subtract.Y;
    }

    public void mult( Pose2d multiply ) {
        X *= multiply.X;
        Y *= multiply.Y;
    }

    public void div( Pose2d divide ) {
        X /= divide.X;
        Y /= divide.Y;
    }

    public double getMagnitude() {
        return Math.sqrt(X * X + Y * Y);
    }

    public double dot(Pose2d other) {
        return X * other.X + Y * other.Y;
    }

    public void rotateRadians(double radians) {
        double x2 = X * cos(radians) - Y * sin(radians);
        double y2 = X * sin(radians) + Y * cos(radians);
        X = x2;
        Y = y2;
    }

    public double getAngleRadians() {
        return Math.atan2(Y, X);
    }

    public double getAngleRadians(Pose2d other) {
        return Math.atan2(other.Y, other.X) - Math.atan2(Y, X);
    }

    public void normalize() {
        double magnitude = this.getMagnitude();
        X /= magnitude;
        Y /= magnitude;
    }

    public void mult(double factor) {
        X *= factor;
        Y *= factor;
    }

    public void div(double divisor) {
        X /= divisor;
        Y /= divisor;
    }

    public void capAt1() {
        if (X > 1) {
            X /= X;
            Y /= X;
        }
        if (Y > 1) {
            Y /= Y;
            X /= Y;
        }
        if (X < -1) {
            X /= Math.abs(X);
            Y /= Math.abs(X);
        }
        if (Y < -1) {
            Y /= Math.abs(Y);
            X /= Math.abs(Y);
        }
    }

    //Static methods

    public static Pose2d fromAngleRadians(double radians) {
        return new Pose2d(cos(radians), sin(radians));
    }

    public static Pose2d add(Pose2d add1, Pose2d add2) {
        return new Pose2d(add1.X + add2.X, add1.Y + add2.Y, add1.heading + add2.heading);
    }

    public static Pose2d sub(Pose2d sub1, Pose2d sub2) {
        return new Pose2d(sub1.X - sub2.X, sub1.Y - sub2.Y, sub1.heading - sub2.heading );
    }

    public static Pose2d mult(Pose2d mult1, Pose2d mult2) {
        return new Pose2d(mult1.X * mult2.X, mult1.Y * mult2.Y, mult1.heading * mult2.heading);
    }

    public static Pose2d div(Pose2d div1, Pose2d div2) {
        return new Pose2d(div1.X / div2.X, div1.Y / div2.Y, div1.heading / div2.heading);
    }

    public static Pose2d add(Pose2d pose2d, double add) {
        return new Pose2d(pose2d.X + add, pose2d.Y + add, pose2d.heading + add);
    }

    public static Pose2d sub(Pose2d pose2d, double sub) {
        return new Pose2d(pose2d.X - sub, pose2d.Y - sub, pose2d.heading - sub);
    }

    public static Pose2d div(Pose2d dividend, double divisor) {
        return new Pose2d(dividend.X / divisor, dividend.Y / divisor, dividend.heading / divisor);
    }

    public static Pose2d mult(Pose2d a, double b) {
        return new Pose2d(a.X / b, a.Y / b, a.heading / b);
    }

    public static Vector2d Vector2d(Pose2d Pose2d){
        return new Vector2d(Pose2d.X, Pose2d.Y);
    }

    public static double getMagnitude(Pose2d pose2d) {
        return Math.sqrt(pose2d.X * pose2d.X + pose2d.Y * pose2d.Y);
    }
}
