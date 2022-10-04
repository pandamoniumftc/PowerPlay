package org.firstinspires.ftc.teamcode.SCPML.util.spline;

import org.firstinspires.ftc.teamcode.SCPML.util.Vector2d;

import java.util.ArrayList;
import java.util.List;

public class BezierCurve {

    List<Vector2d> Vector2dList = new ArrayList<>();

    public Vector2d A, B, end = new Vector2d();

    public int steps;


    public BezierCurve(Vector2d a, Vector2d b, Vector2d End, int Steps) {
        steps = Steps;
        for (int i = 0; i < Steps; i++ ) {
            Vector2d point = getMiddlePoint(getMiddlePoint(a, b, i / Steps), getMiddlePoint(b, End, i/Steps), i/Steps);

            Vector2dList.add(i, point);
        }
        A = a;
        B = b;
        end = End;
    }

    private Vector2d getMiddlePoint(Vector2d A, Vector2d B, double distance) {
        double angle = Math.atan(B.Y - A.Y / B.X - A.X);

        double pointDistance = Math.sqrt( Math.pow(B.X-A.X, 2) + Math.pow(B.Y-A.Y,2));

        distance *= pointDistance;

        return Vector2d.add(A, new Vector2d(Math.asin(angle) / distance, Math.acos(angle) / distance));
    }
}
