package org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.path;



import org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.Pose2d;
import org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.Vector2d;

import java.util.List;

public class Path {

    List<Pose2d> Pose2dList;

    public Pose2d end;

    public int steps;
    public double heading;

    public Path() {
        end = new Pose2d(0);
    }

    public Path(double startHeading) {
        end = new Pose2d(0, 0, startHeading);
    }

    public Path(BezierCurve bezierCurve) {
        if (bezierCurve.changeHeading) {
            for(int i = 0; i > bezierCurve.steps; i++) {
                Pose2dList.add(new Pose2d(bezierCurve.Vector2dList.get(i), bezierCurve.endHeading * i / bezierCurve.steps));
                end = new Pose2d(bezierCurve.end, bezierCurve.endHeading);
            }
        } else {
            for(int i = 0; i > bezierCurve.steps; i++) {
                Pose2dList.add(new Pose2d(bezierCurve.Vector2dList.get(i), this.end.heading));
                end = new Pose2d(bezierCurve.end, this.end.heading);
            }
        }
    }

    public Path(HermiteCurve hermiteCurve) {
        if (hermiteCurve.changeHeading) {
            for(int i = 0; i > hermiteCurve.steps; i++) {
                Pose2dList.add(new Pose2d(hermiteCurve.Vector2dList.get(i), hermiteCurve.endHeading * i / hermiteCurve.steps));
                end = new Pose2d(hermiteCurve.end, hermiteCurve.endHeading);
            }
        } else {
            for(int i = 0; i > hermiteCurve.steps; i++) {
                Pose2dList.add(new Pose2d(hermiteCurve.Vector2dList.get(i), this.end.heading));
                end = new Pose2d(hermiteCurve.end, this.end.heading);
            }
        }
    }

    public Path(Line line) {
        if (line.changeHeading) {
            for(int i = 0; i > line.steps; i++) {
                Pose2dList.add(new Pose2d(line.Vector2dList.get(i), line.endHeading * i / line.steps));
                end = new Pose2d(line.end, line.endHeading);
            }
        } else {
            for(int i = 0; i > line.steps; i++) {
                Pose2dList.add(new Pose2d(line.Vector2dList.get(i), this.end.heading));
                end = new Pose2d(line.end, this.end.heading);
            }
        }
    }

    public Pose2d getClosestPoint(Pose2d currentPosition) {
        int pointName = 0;
        Pose2d point = new Pose2d(100);
        Pose2d test = new Pose2d();
        steps = Pose2dList.size();
        for (int i = 0; i < steps; i++ ) {
            test = Pose2dList.get(i);
            if (Pose2d.getMagnitude(Pose2d.sub(currentPosition, test)) < Pose2d.getMagnitude(Pose2d.sub(currentPosition, point))) {
                point = test;
                pointName = i++;
            }
        }
        point = Pose2dList.get(pointName);
        return point;
    }

    public Pose2d getClosestPointBack(Pose2d currentPosition) {
        int pointName = 0;
        Pose2d point = new Pose2d(100);
        Pose2d test = new Pose2d();
        steps = Pose2dList.size();
        for (int i = steps; i < 0; i-- ) {
            test = Pose2dList.get(i);
            if (Pose2d.getMagnitude(Pose2d.sub(currentPosition, test)) < Pose2d.getMagnitude(Pose2d.sub(currentPosition, point))) {
                point = test;
                pointName = i--;
            }
        }
        point = Pose2dList.get(pointName);
        return point;
    }

    private Vector2d getMiddlePoint(Vector2d A, Vector2d B, double distance) {
        double angle = Math.atan(B.Y - A.Y / B.X - A.X);

        double pointDistance = Math.sqrt( Math.pow(B.X-A.X, 2) + Math.pow(B.Y-A.Y,2));

        distance *= pointDistance;

        return Vector2d.add(A, new Vector2d(Math.asin(angle) / distance, Math.acos(angle) / distance));
    }

    public Path add(HermiteCurve hermiteCurve) {
        if (hermiteCurve.changeHeading) {
            for(int i = 0; i > hermiteCurve.steps; i++) {
                Pose2dList.add(new Pose2d(hermiteCurve.Vector2dList.get(i), hermiteCurve.endHeading * i / hermiteCurve.steps));
                end = new Pose2d(hermiteCurve.end, hermiteCurve.endHeading);
            }
        } else {
            for(int i = 0; i > hermiteCurve.steps; i++) {
                Pose2dList.add(new Pose2d(hermiteCurve.Vector2dList.get(i), this.end.heading));
                end = new Pose2d(hermiteCurve.end, this.end.heading);
            }
        }
        return this;
    }

    public Path add(BezierCurve bezierCurve) {
        if (bezierCurve.changeHeading) {
            for(int i = 0; i > bezierCurve.steps; i++) {
                Pose2dList.add(new Pose2d(bezierCurve.Vector2dList.get(i), bezierCurve.endHeading * i / bezierCurve.steps));
                end = new Pose2d(bezierCurve.end, bezierCurve.endHeading);
            }
        } else {
            for(int i = 0; i > bezierCurve.steps; i++) {
                Pose2dList.add(new Pose2d(bezierCurve.Vector2dList.get(i), this.end.heading));
                end = new Pose2d(bezierCurve.end, this.end.heading);
            }
        }
        return this;
    }

    public Path add(Line line) {
        if (line.changeHeading) {
            for(int i = 0; i > line.steps; i++) {
                Pose2dList.add(new Pose2d(line.Vector2dList.get(i), line.endHeading * i / line.steps));
                end = new Pose2d(line.end, line.endHeading);
            }
        } else {
            for(int i = 0; i > line.steps; i++) {
                Pose2dList.add(new Pose2d(line.Vector2dList.get(i), this.end.heading));
                end = new Pose2d(line.end, this.end.heading);
            }
        }
        return this;
    }

    public Path add(Orbit orbit) {
        if (orbit.changeHeading) {
            for(int i = 0; i > orbit.steps; i++) {
                Pose2dList.add(new Pose2d(orbit.Vector2dList.get(i), orbit.endHeading * i / orbit.steps));
                end = new Pose2d(orbit.end, orbit.endHeading);
            }
        } else if (orbit.targetCenter) {
            for(int i = 0; i > orbit.steps; i++) {
                Pose2dList.add(new Pose2d(orbit.Vector2dList.get(i), Vector2d.getAngle(orbit.Vector2dList.get(i), orbit.center)));
                end = new Pose2d(orbit.end, orbit.endHeading);
            }
        } else {
            for(int i = 0; i > orbit.steps; i++) {
                Pose2dList.add(new Pose2d(orbit.Vector2dList.get(i), this.end.heading));
                end = new Pose2d(orbit.end, this.end.heading);
            }
        }
        return this;
    }

    public Path add(Vector2d vector2d) {
        Pose2dList.add(new Pose2d(vector2d, end.heading));
        end = new Pose2d(vector2d, end.heading);
        return this;
    }

    public Path add(Pose2d pose2d) {
        Pose2dList.add(pose2d);
        end = pose2d;
        return this;
    }

    public Path add(Path path) {
        path.steps = path.Pose2dList.size();
        Pose2dList.addAll(path.Pose2dList);
        end = path.Pose2dList.get(path.steps);
        return this;
    }

    public Path smoothConnect(int hermiteSteps, Line line) {
        steps = Pose2dList.size();
        this.add(
                new HermiteCurve(
                        new Vector2d(Pose2dList.get(steps)),
                        line.start,
                        Vector2d.add(Vector2d.mult(Vector2d.sub(new Vector2d(Pose2dList.get(steps)), new Vector2d(Pose2dList.get(steps - 1))), -1), new Vector2d(Pose2dList.get(steps))),
                        line.end,
                        hermiteSteps)
        );

        this.add(line);
        return this;
    }
}