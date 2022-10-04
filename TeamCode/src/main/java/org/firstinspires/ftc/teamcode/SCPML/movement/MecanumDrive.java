package org.firstinspires.ftc.teamcode.SCPML.movement;

import static java.lang.Math.asin;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.SCPML.util.*;
import org.firstinspires.ftc.teamcode.SCPML.util.path.Path;


public class MecanumDrive {

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private Ultrasonic frontRightUltrasonic, frontLeftUltrasonic, rightUltrasonic, leftUltrasonic;
    public BNO055IMU imu;
    public double startingAngle;

    public Pose2d currentPosition = new Pose2d();

    //Constants
    //Ultrasonic distances from the top
    public double frontRightUltrasonicDistance = 1;
    public double frontLeftUltrasonicDistance = 1;
    public double rightUltrasonicDistance = 1;
    public double leftUltrasonicDistance = 1;

    //Ultrasonic distance from the side
    public double frontRightUltrasonicAnglePitch = 0;
    public double frontLeftUltrasonicAnglePitch = 0;
    public double rightUltrasonicAngleRoll = 0;
    public double leftUltrasonicAngleRoll = 0;
    public double frontRightUltrasonicDistancePitch = 1;
    public double frontLeftUltrasonicDistancePitch = 1;
    public double rightUltrasonicDistanceRoll = 1;
    public double leftUltrasonicDistanceRoll = 1;


    public MecanumDrive(LinearOpMode opMode) {

        leftFront = opMode.hardwareMap.get(DcMotorEx.class, "flm");
        leftRear = opMode.hardwareMap.get(DcMotorEx.class, "blm");
        rightRear = opMode.hardwareMap.get(DcMotorEx.class, "brm");
        rightFront = opMode.hardwareMap.get(DcMotorEx.class, "frm");
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        //Ultrasonic
        frontRightUltrasonic = new Ultrasonic(opMode.hardwareMap.get(AnalogInput.class, "fr_ultrasonic"));
        frontLeftUltrasonic = new Ultrasonic(opMode.hardwareMap.get(AnalogInput.class, "fl_ultrasonic"));
        rightUltrasonic = new Ultrasonic(opMode.hardwareMap.get(AnalogInput.class, "br_ultrasonic"));
        leftUltrasonic = new Ultrasonic(opMode.hardwareMap.get(AnalogInput.class, "bl_ultrasonic"));

        //IMU init
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

    }

    public void trackPosition() {

    }

    // ToDo: Switch imu axes to align with orientation of robot

    /*
                            /\
                             |   x-axis / Pitch
                             |
                             |
     ______________________________________________
    |                                             |
    |                                             |
    |                                             |
    |                                             |
    |                Control Hub                  |  /_____ y-axis / Roll
    |                                             |  \
    |                                             |
    |                                             |
    |_____________________________________________|

     */

    public double ultrasonicFront;
    public double ultrasonicSide;
    public double distanceToPoint;
    public double pitchRollAngle;
    public double yawAngle;
    public double distanceToPointAngleOffset;
    Vector2d UltrasonicPosition = new Vector2d();
    public int ultrasonicSideMultiplier;

    public Vector2d UltrasonicPositionRight() {
        trackPosition();

        yawAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        if (currentPosition.heading >= 0){
            ultrasonicFront = frontLeftUltrasonic.getDistance();

            //Pitch correction
            pitchRollAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            distanceToPoint = sqrt(pow(frontLeftUltrasonicDistancePitch, 2 ) + pow(ultrasonicFront, 2)
                    - 2 * frontLeftUltrasonicDistancePitch * ultrasonicFront * cos(frontLeftUltrasonicAnglePitch));
            ultrasonicFront = distanceToPoint * cos(pitchRollAngle - asin(ultrasonicFront * sin(frontLeftUltrasonicAnglePitch) / distanceToPoint));

            //Angle Correction

            distanceToPoint = sqrt( pow(frontLeftUltrasonicDistance, 2) + pow( ultrasonicFront, 2) );
            UltrasonicPosition.Y = distanceToPoint * ( yawAngle - atan(ultrasonicFront / frontLeftUltrasonicDistance) );

        } else {
            ultrasonicFront = frontRightUltrasonic.getDistance();

            //Pitch correction
            pitchRollAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            distanceToPoint = sqrt(pow(frontRightUltrasonicDistancePitch, 2 ) + pow(ultrasonicFront, 2)
                    - 2 * frontRightUltrasonicDistancePitch * ultrasonicFront * cos(frontRightUltrasonicAnglePitch));
            ultrasonicFront = distanceToPoint * cos(pitchRollAngle - asin(ultrasonicFront * sin(frontRightUltrasonicAnglePitch) / distanceToPoint));

            //Angle Correction

            distanceToPoint = sqrt( pow(frontRightUltrasonicDistance, 2) + pow( ultrasonicFront, 2) );
            UltrasonicPosition.Y = distanceToPoint * ( yawAngle - atan(ultrasonicFront / frontRightUltrasonicDistance) );
        }
        ultrasonicSide = rightUltrasonic.getDistance();

        //Roll correction
        pitchRollAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        distanceToPoint = sqrt(pow(rightUltrasonicDistanceRoll, 2 ) + pow(ultrasonicSide, 2)
                - 2 * rightUltrasonicDistanceRoll * ultrasonicFront * cos(rightUltrasonicAngleRoll));
        ultrasonicFront = distanceToPoint * cos(pitchRollAngle - asin(ultrasonicSide * sin(rightUltrasonicAngleRoll) / distanceToPoint));

        //Angle Correction

        distanceToPoint = sqrt( pow(rightUltrasonicDistance, 2) + pow( ultrasonicSide, 2) );
        UltrasonicPosition.X = distanceToPoint * ( yawAngle - atan(ultrasonicSide / rightUltrasonicDistance) );

        return UltrasonicPosition;
    }

    public Vector2d trackPositionUltrasonicLeft() {
        trackPosition();

        ultrasonicSideMultiplier = -1;

        yawAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        if (currentPosition.heading >= 0){
            ultrasonicFront = frontLeftUltrasonic.getDistance();
            //Pitch correction
            pitchRollAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            distanceToPoint = sqrt(pow(frontLeftUltrasonicDistancePitch, 2 ) + pow(ultrasonicFront, 2)
                    - 2 * frontLeftUltrasonicDistancePitch * ultrasonicFront * cos(frontLeftUltrasonicAnglePitch));
            ultrasonicFront = distanceToPoint * cos(pitchRollAngle - asin(ultrasonicFront * sin(frontLeftUltrasonicAnglePitch) / distanceToPoint));

            //Angle Correction

            distanceToPoint = sqrt( pow(frontLeftUltrasonicDistance, 2) + pow( ultrasonicFront, 2) );
            UltrasonicPosition.Y = distanceToPoint * ( yawAngle - atan(ultrasonicFront / frontLeftUltrasonicDistance) );

        } else {
            ultrasonicFront = frontRightUltrasonic.getDistance();

            //Pitch correction
            pitchRollAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            distanceToPoint = sqrt(pow(frontRightUltrasonicDistancePitch, 2 ) + pow(ultrasonicFront, 2)
                    - 2 * frontRightUltrasonicDistancePitch * ultrasonicFront * cos(frontRightUltrasonicAnglePitch));
            ultrasonicFront = distanceToPoint * cos(pitchRollAngle - asin(ultrasonicFront * sin(frontRightUltrasonicAnglePitch) / distanceToPoint));

            //Angle Correction

            distanceToPoint = sqrt( pow(frontRightUltrasonicDistance, 2) + pow( ultrasonicFront, 2) );
            UltrasonicPosition.Y = distanceToPoint * ( yawAngle - atan(ultrasonicFront / frontRightUltrasonicDistance) );
        }
        ultrasonicSide = leftUltrasonic.getDistance();

        //Roll correction
        pitchRollAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        distanceToPoint = sqrt(pow(leftUltrasonicDistanceRoll, 2 ) + pow(ultrasonicSide, 2)
                - 2 * leftUltrasonicDistanceRoll * ultrasonicFront * cos(leftUltrasonicAngleRoll));
        ultrasonicFront = distanceToPoint * cos(pitchRollAngle - asin(ultrasonicSide * sin(leftUltrasonicAngleRoll) / distanceToPoint));

        //Angle Correction

        distanceToPoint = sqrt( pow(leftUltrasonicDistance, 2) + pow( ultrasonicSide, 2) );
        UltrasonicPosition.X = distanceToPoint * ( yawAngle - atan(ultrasonicSide / leftUltrasonicDistance) );

        return UltrasonicPosition;
    }

    Vector2d ultrasonicPositionSet = new Vector2d();
    public Vector2d rotateUltrasonic(Vector2d ultrasonicPosition) {
        trackPosition();
        if (currentPosition.heading >= 0) {
            if (currentPosition.heading <= 45) {
                ultrasonicPositionSet.Y = ultrasonicPosition.Y;
                ultrasonicPositionSet.X = ultrasonicPosition.X * ultrasonicSideMultiplier;
            } else if (currentPosition.heading <= 135) {
                ultrasonicPositionSet.Y = ultrasonicPosition.X * ultrasonicSideMultiplier;
                ultrasonicPositionSet.X = ultrasonicPosition.Y;
            } else {
                ultrasonicPositionSet.Y = -ultrasonicPosition.Y;
                ultrasonicPositionSet.X = -ultrasonicPosition.X * ultrasonicSideMultiplier;
            }
        } else {
            if (currentPosition.heading >= 45) {
                ultrasonicPositionSet.Y = ultrasonicPosition.Y;
                ultrasonicPositionSet.X = ultrasonicPosition.X * ultrasonicSideMultiplier;
            } else if (currentPosition.heading >= 135) {
                ultrasonicPositionSet.Y = ultrasonicPosition.X * ultrasonicSideMultiplier;
                ultrasonicPositionSet.X = -ultrasonicPosition.Y;
            } else {
                ultrasonicPositionSet.Y = -ultrasonicPosition.Y;
                ultrasonicPositionSet.X = -ultrasonicPosition.X * ultrasonicSideMultiplier;
            }
        }

        ultrasonicSideMultiplier = 1;
        return ultrasonicPositionSet;
    }

    //Movement stuff

    public void setPositionTo(Vector2d positionSet) {

    }

    public void setPositionTo(Pose2d positionSet) {

    }

    public void turn(double heading){
        leftFront.setPower(Range.clip( heading, -1, 1 ));
        leftRear.setPower(Range.clip( heading, -1, 1 ));
        rightFront.setPower(Range.clip( -heading, -1, 1 ));
        rightRear.setPower(Range.clip( -heading, -1, 1 ));
    }

    public void move(Vector2d move) {
        move.capAt1();
        leftFront.setPower(Range.clip(move.X + move.Y, -1, 1));
        leftRear.setPower(Range.clip(-move.X + move.Y, -1, 1));
        rightRear.setPower(Range.clip(move.X + move.Y, -1, 1));
        rightFront.setPower(Range.clip(-move.X + move.Y, -1, 1));
    }

    public void move(Vector2d move, double speed) {
        move.capAt1();
        leftFront.setPower(Range.clip((move.X + move.Y) * speed, -1, 1));
        leftRear.setPower(Range.clip((-move.X + move.Y) * speed, -1, 1));
        rightRear.setPower(Range.clip((move.X + move.Y) * speed, -1, 1));
        rightFront.setPower(Range.clip((-move.X + move.Y) * speed, -1, 1));
    }

    public void move(Pose2d move) {
        move.capAt1();
        leftFront.setPower(Range.clip(move.X + move.Y + move.heading, -1, 1));
        leftRear.setPower(Range.clip(-move.X + move.Y + move.heading, -1, 1));
        rightRear.setPower(Range.clip(move.X + move.Y - move.heading, -1, 1));
        rightFront.setPower(Range.clip(-move.X + move.Y - move.heading, -1, 1));
    }

    public void move(Pose2d move, double speed) {
        move.capAt1();
        leftFront.setPower(Range.clip(move.X + move.Y + move.heading * speed, -1, 1));
        leftRear.setPower(Range.clip(-move.X + move.Y + move.heading * speed, -1, 1));
        rightRear.setPower(Range.clip(move.X + move.Y - move.heading * speed, -1, 1));
        rightFront.setPower(Range.clip(-move.X + move.Y - move.heading * speed, -1, 1));
    }

    public void move(double speed) {
        leftFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(speed);
        rightFront.setPower(speed);
    }

    public void stopMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }

    public void moveTowards(Vector2d target) {
        trackPosition();
        Vector2d error = Vector2d.sub( new Vector2d(currentPosition), target);
        move(error);
    }

    public void moveTowards(Vector2d target, double speed) {
        trackPosition();
        Vector2d error = Vector2d.sub( new Vector2d(currentPosition), target);
        move(error, speed);
    }

    public void moveTowards(Pose2d target, double speed) {
        trackPosition();
        double headingError;
        if (Math.abs(currentPosition.heading - target.heading) >= Math.abs(currentPosition.heading - target.heading - 360)){
            headingError = currentPosition.heading - target.heading - 360;
        } else {
            headingError = currentPosition.heading - target.heading;
        }
        Pose2d error = Pose2d.sub(currentPosition, target);
        move( new Pose2d( new Vector2d(error), headingError), speed);
    }

    public void setMotorPowers(double leftfront, double leftrear, double rightrear, double rightfront) {
        leftFront.setPower(leftfront);
        leftRear.setPower(leftrear);
        rightRear.setPower(rightrear);
        rightFront.setPower(rightfront);
    }

    public void turnTo(double heading) {
        PID turnPID = new PID(PID.Type.turn);
        double headingError = 1;
        while (headingError != 0){
            trackPosition();
            if (Math.abs(currentPosition.heading - heading) >= Math.abs(currentPosition.heading - heading - 360)){
                headingError = currentPosition.heading - heading - 360;
            } else {
                headingError = currentPosition.heading - heading;
            }
            turnPID.pid(headingError);
            turn(turnPID.pidout);
        }
        stopMotors();
    }

    public void lineTo(Vector2d target) {
        NanoClock Time = new NanoClock();
        Vector2d error = new Vector2d();
        PID pidX = new PID(PID.Type.move);
        PID pidY = new PID(PID.Type.move);
        while (error.X != 0 & error.Y != 0) {
            trackPosition();
            error = Vector2d.sub( Pose2d.Vector2d(currentPosition), target);
            pidX.pid(error.X);
            pidY.pid(error.Y);
            move(new Vector2d(pidX.pidout, pidY.pidout));
        }
        stopMotors();
    }

    public void lineToSplineHeading(Pose2d target) {
        NanoClock Time = new NanoClock();
        Pose2d error = new Pose2d(1);
        PID pidX = new PID(PID.Type.move);
        PID pidY = new PID(PID.Type.move);
        PID pidHeading = new PID(PID.Type.turn);
        double headingError = 0;
        while (error.X != 0 & error.Y != 0 & headingError != 0) {
            trackPosition();
            error = Pose2d.sub(currentPosition, target);
            pidX.pid(error.X);
            pidY.pid(error.Y);
            pidHeading.pid(error.heading);

            //Heading
            if (Math.abs(currentPosition.heading - target.heading) >= Math.abs(currentPosition.heading - target.heading - 360)) {
                headingError = currentPosition.heading - target.heading - 360;
            } else {
                headingError = Math.abs(currentPosition.heading - target.heading);
            }
            pidHeading.pid(headingError);
            move(new Pose2d(pidX.pidout, pidY.pidout, pidHeading.pidout));

        }
        stopMotors();
    }

    public void spline(Path spline) {
        PID error = new PID(PID.Type.move);
        Pose2d Error = new Pose2d(1);
        while (Error.getMagnitude() != 0 && Error.heading != 0) {
            Error = Pose2d.sub(currentPosition, spline.end);
            error.pid(Error.getMagnitude());
            moveTowards(spline.getClosestPoint(currentPosition), error.pidout);
        }
        stopMotors();
    }

    public void splineConstantSpeed(Path spline, double speed) {
        Pose2d Error = new Pose2d(1);
        while (Error.getMagnitude() != 0 && Error.heading != 0) {
            Error = Pose2d.sub(currentPosition, spline.end);
            moveTowards(spline.getClosestPoint(currentPosition), speed);
        }
        stopMotors();
    }

    public void splineBack(Path spline) {
        PID error = new PID(PID.Type.move);
        Pose2d Error = new Pose2d(1);
        while (Error.getMagnitude() != 0 && Error.heading != 0) {
            Error = Pose2d.sub(currentPosition, spline.end);
            error.pid(Error.getMagnitude());
            moveTowards(spline.getClosestPointBack(currentPosition), error.pidout);
        }
        stopMotors();
    }

    public void splineBackConstantSpeed(Path spline, double speed) {
        Pose2d Error = new Pose2d(1);
        while (Error.getMagnitude() != 0 && Error.heading != 0) {
            Error = Pose2d.sub(currentPosition, spline.end);
            moveTowards(spline.getClosestPointBack(currentPosition), speed);
        }
        stopMotors();
    }
}