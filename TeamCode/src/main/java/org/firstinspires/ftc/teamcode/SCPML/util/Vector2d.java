package org.firstinspires.ftc.teamcode.SCPML.util;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class Vector2d extends Object {

    public double X = 0;
    public double Y = 0;

    public Vector2d(double x, double y) {
        X = x;
        Y = y;
    }

    public Vector2d(double XAndY) {
        X = XAndY;
        Y = XAndY;
    }

    public Vector2d(Pose2d pose2d) {
        X = pose2d.X;
        Y = pose2d.Y;
    }

    public Vector2d() {}

    /**
     * adds the add Vector2d X and Y values to the current A and Y values
     * @param add
     */
    public void add(Vector2d add) {
        X += add.X;
        Y += add.Y;
    }

    public void add(double add) {
        X += add;
        Y += add;
    }

    /**
     * subtracts the sub Vector2d X and Y values from the current X and Y value
     * @param sub
     */
    public void sub( Vector2d sub ) {
        X -= sub.X;
        Y -= sub.Y;
    }

    public void mult( Vector2d multiply ) {
        X *= multiply.X;
        Y *= multiply.Y;
    }

    public void div( Vector2d divide ) {
        X /= divide.X;
        Y /= divide.Y;
    }

    /**
     * @return The distance from this point to 0
     */
    public double getMagnitude() {
        return Math.sqrt(X * X + Y * Y);
    }

    public double dot(Vector2d other) {
        return X * other.X + Y * other.Y;
    }

    /**
     * Rotates the coordinates in an ark
     * @param radians the value that this point is rotated by
     */
    public void rotateRadians(double radians) {
        double x2 = X * cos(radians) - Y * sin(radians);
        double y2 = X * sin(radians) + Y * cos(radians);
        X = x2;
        Y = y2;
    }

    /**
     * @return the angle in radians from 0 counterclockwise to this point
     */
    public double getAngleRadians() {
        return Math.atan2(Y, X);
    }

    /**
     * @param other Vector2d used in angle
     * @return the angle in radians from other clockwise to this point
     */
    public double getAngleRadians(Vector2d other) {
        return Math.atan2(other.Y, other.X) - Math.atan2(Y, X);
    }

    /**
     * Sets magnitude to 1 while maintaining direction
     */
    public void normalize() {
        double magnitude = this.getMagnitude();
        X /= magnitude;
        Y /= magnitude;
    }

    /**
     * @param factor factor that is multiplied by both this X and Y separately
     */
    public void mult(double factor) {
        X *= factor;
        Y *= factor;
    }

    /**
     * @param divisor number that both X and Y are divided by separately
     */
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

    public static Vector2d fromAngleRadians(double radians) {
        return new Vector2d(cos(radians), sin(radians));
    }

    public static Vector2d add(Vector2d add1, Vector2d add2) {
        return new Vector2d(add1.X + add2.X, add1.Y + add2.Y );
    }

    public static Vector2d add(Vector2d add1, Vector2d add2, Vector2d add3, Vector2d add4) {
        return new Vector2d(add1.X + add2.X + add3.X + add4.X, add1.Y + add2.Y + add3.Y + add4.Y);
    }

    public static Vector2d sub(Vector2d sub1, Vector2d sub2) {
        return new Vector2d(sub1.X - sub2.X, sub1.Y - sub2.Y );
    }

    public static Vector2d mult(Vector2d mult1, Vector2d mult2) {
        return new Vector2d(mult1.X * mult2.X, mult1.Y * mult2.Y );
    }

    public static Vector2d div(Vector2d div1, Vector2d div2) {
        return new Vector2d(div1.X / div2.X, div1.Y / div2.Y );
    }

    public static Vector2d add(Vector2d Vector2d, double add) {
        return new Vector2d(Vector2d.X + add, Vector2d.Y + add);
    }

    public static Vector2d sub(Vector2d Vector2d, double sub) {
        return new Vector2d(Vector2d.X - sub, Vector2d.Y - sub);
    }

    public static Vector2d div(Vector2d dividend, double divisor) {
        return new Vector2d(dividend.X / divisor, dividend.Y / divisor);
    }

    public static Vector2d mult(Vector2d a, double b) {
        return new Vector2d(a.X / b, a.Y / b);
    }

    public static Pose2d Pose2d(Vector2d Vector2d){
        return new Pose2d(Vector2d.X, Vector2d.Y);
    }

    public static double getMagnitude(Vector2d vector2d) {
        return Math.sqrt(vector2d.X * vector2d.X + vector2d.Y * vector2d.Y);
    }

    public static double getAngle(Vector2d a, Vector2d b) {
        return Vector2d.sub(a, b).getAngleRadians();
    }
}