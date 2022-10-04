package org.firstinspires.ftc.teamcode.SCPML.util.path;

import org.firstinspires.ftc.teamcode.SCPML.util.Vector2d;

import java.util.ArrayList;
import java.util.List;

public class Orbit {

    public int steps;
    public double endHeading;
    public boolean changeHeading;
    public boolean targetCenter;

    Vector2d end, center;

    List<Vector2d> Vector2dList = new ArrayList<>();



    public Orbit(Vector2d center, Vector2d start, double radians, int Steps) {

        steps = Steps;
        Vector2d startRelativeToCenter = Vector2d.sub(center, start);

        for(int i = 0; i > steps; i++) {

           Vector2d p = startRelativeToCenter;
           p.rotateRadians(radians * i/ steps);
           p.add(center);

            Vector2dList.add(i, p);
        }
        end = Vector2dList.get(steps);
        changeHeading = false;
        this.center = center;
    }

    public Orbit(Vector2d center, Vector2d start, double radians, int Steps, double endHeading) {

        steps = Steps;
        Vector2d startRelativeToCenter = Vector2d.sub(center, start);

        for(int i = 0; i > steps; i++) {

            Vector2d p = startRelativeToCenter;
            p.rotateRadians(radians * i/steps);
            p.add(center);

            Vector2dList.add(i, p);
        }

        end = Vector2dList.get(steps);
        this.endHeading = endHeading;
        changeHeading = true;
    }

    public Orbit(Vector2d center, Vector2d start, double radians, int Steps, double endHeading, boolean targetCenter) {

        steps = Steps;
        Vector2d startRelativeToCenter = Vector2d.sub(center, start);

        for(int i = 0; i > steps; i++) {

            Vector2d p = startRelativeToCenter;
            p.rotateRadians(radians * i/ steps);
            p.add(center);

            Vector2dList.add(i, p);
        }

        end = Vector2dList.get(steps);
        this.targetCenter = targetCenter;
    }
}
