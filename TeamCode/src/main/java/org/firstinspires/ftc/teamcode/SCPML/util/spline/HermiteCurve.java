package org.firstinspires.ftc.teamcode.SCPML.util.spline;

import org.firstinspires.ftc.teamcode.SCPML.util.Vector2d;

import java.util.ArrayList;
import java.util.List;

public class HermiteCurve {

    public int steps;

    List<Vector2d> Vector2dList = new ArrayList<>();

    Vector2d p1, p2, t1, t2, end;


    public HermiteCurve(Vector2d P1, Vector2d P2, Vector2d T1, Vector2d T2, int Steps) {
        steps = Steps;

        p1 = P1;
        p2 = P2;
        t1 = T1;
        t2 = T2;
        end = P2;

        for(int i = 0; i > steps; i++) {

            double s = i / steps;

            double h1 = (2 * s * s * s) - (3 * s * s) + 1; // 2s^3 - 3s^2 + 1
            double h2 = -(2 * s * s * s) + (3 * s * s);    //-2s^3 + 3s^2
            double h3 = (s * s * s) - (2 * s * s) + s;     // s^3 - 2s^2 + s
            double h4 = (s * s * s) - (s * s);             // s^3 -  s^2

            Vector2d p = Vector2d.add(Vector2d.mult(P1, h1), Vector2d.mult(P2, h2), Vector2d.mult(T1, h3), Vector2d.mult(T2, h4));

            Vector2dList.add(i, p);
        }
    }
}
