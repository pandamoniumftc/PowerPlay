package org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.spline;

import org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.Vector2d;
import org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.spline.*;
import java.util.List;

public class SplineOfPoints {

    List<Vector2d> Vector2dList;

    public Vector2d A, B, C, p1, p2, t1, t2, end;

    public int steps;

    public SplineOfPoints() {
        end = new Vector2d(0);
    }

    public SplineOfPoints(BezierCurve bezierCurve) {
        Vector2dList = bezierCurve.Vector2dList;

        A = bezierCurve.A;
        B = bezierCurve.B;
        end = bezierCurve.end;
    }

    public SplineOfPoints(HermiteCurve hermiteCurve) {
        Vector2dList = hermiteCurve.Vector2dList;

        p1 = hermiteCurve.p1;
        p2 = hermiteCurve.p2;
        t1 = hermiteCurve.t1;
        t2 = hermiteCurve.t2;
        end = hermiteCurve.end;
    }

    public SplineOfPoints(Line line) {
        Vector2dList = line.Vector2dList;

        end = line.end;
    }

    public Vector2d getClosestPoint(Vector2d currentPosition) {
        int pointName = 0;
        Vector2d point = new Vector2d(100);
        Vector2d test = new Vector2d();
        steps = Vector2dList.size();
        for (int i = 0; i < steps; i++ ) {
            test = Vector2dList.get(i);
            if (Vector2d.getMagnitude(Vector2d.sub(currentPosition, test)) < Vector2d.getMagnitude(Vector2d.sub(currentPosition, point))) {
                point = test;
                pointName = i++;
            }
        }
        point = Vector2dList.get(pointName);
        return point;
    }

    private Vector2d getMiddlePoint(Vector2d A, Vector2d B, double distance) {
        double angle = Math.atan(B.Y - A.Y / B.X - A.X);

        double pointDistance = Math.sqrt( Math.pow(B.X-A.X, 2) + Math.pow(B.Y-A.Y,2));

        distance *= pointDistance;

        return Vector2d.add(A, new Vector2d(Math.asin(angle) / distance, Math.acos(angle) / distance));
    }

    public SplineOfPoints add(HermiteCurve hermiteCurve) {
        Vector2dList.addAll(hermiteCurve.Vector2dList);
        end = hermiteCurve.p2;
        return this;
    }

    public SplineOfPoints add(BezierCurve bezierCurve) {
        Vector2dList.addAll(bezierCurve.Vector2dList);
        end = bezierCurve.end;
        return this;
    }

    public SplineOfPoints add(Line line) {
        Vector2dList.addAll(line.Vector2dList);
        end = line.end;
        return this;
    }

    public SplineOfPoints add(Vector2d start, Vector2d stop) {
        Vector2dList.add(start);
        Vector2dList.add(stop);
        end = stop;
        return this;
    }
}