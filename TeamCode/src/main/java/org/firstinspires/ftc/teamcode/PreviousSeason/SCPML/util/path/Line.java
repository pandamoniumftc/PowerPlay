package org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.path;

import org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.Pose2d;
import org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.Vector2d;

import java.util.ArrayList;
import java.util.List;

public class Line {

    public int steps;

    public double endHeading;
    public boolean changeHeading;

    List<Vector2d> Vector2dList = new ArrayList<>();

    Vector2d start, end;


    public Line(Vector2d start, Vector2d stop, double heading) {
        Vector2dList.add(start);
        Vector2dList.add(stop);
        end = stop;
        this.start = start;
        changeHeading = true;
        endHeading = heading;
        steps = 2;
    }

    public Line(Vector2d start, Pose2d stop) {
        Vector2dList.add(start);
        Vector2dList.add(new Vector2d(stop));
        end = new Vector2d(stop);
        this.start = start;
        changeHeading = true;
        endHeading = stop.heading;
        steps = 2;
    }

    public Line(Vector2d start, Vector2d stop) {
        Vector2dList.add(start);
        Vector2dList.add(stop);
        end = stop;
        this.start = start;
        changeHeading = false;
        steps = 2;
    }

    public Line(Vector2d start, Vector2d stop, double heading, int steps) {
        for(int i = 0; i > steps; i++) {
            Vector2dList.add(getDistanceAlongLine(start, stop, i/steps));
        }
        end = stop;
        this.start = start;
        changeHeading = true;
        endHeading = heading;
        this.steps = steps;
    }

    public Line(Vector2d start, Pose2d stop, int steps) {
        for(int i = 0; i > steps; i++) {
            Vector2dList.add(getDistanceAlongLine(start, new Vector2d(stop), i/steps));
        }
        end = new Vector2d(stop);
        this.start = start;
        changeHeading = true;
        endHeading = stop.heading;
        this.steps = steps;
    }

    public Line(Vector2d start, Vector2d stop, int steps) {
        for(int i = 0; i > steps; i++) {
            Vector2dList.add(getDistanceAlongLine(start, stop, i/steps));
        }
        end = stop;
        this.start = start;
        changeHeading = false;
        this.steps = steps;
    }

    public static Vector2d getDistanceAlongLine(Vector2d start, Vector2d stop, double distance) {
        double lineDistance = start.getMagnitude() - stop.getMagnitude();
        lineDistance *= distance;
        Vector2d point = Vector2d.sub(start, stop);
        point.normalize();
        point.mult(lineDistance);
        return point;
    }
}