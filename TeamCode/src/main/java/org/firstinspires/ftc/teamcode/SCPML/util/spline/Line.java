package org.firstinspires.ftc.teamcode.SCPML.util.spline;

import org.firstinspires.ftc.teamcode.SCPML.util.Vector2d;

import java.util.ArrayList;
import java.util.List;

public class Line {

    public int steps;

    List<Vector2d> Vector2dList = new ArrayList<>();

    Vector2d start, end;


    public Line(Vector2d start, Vector2d stop) {
        Vector2dList.add(start);
        Vector2dList.add(stop);
        end = stop;
        this.start = start;
        steps = 2;
    }
}