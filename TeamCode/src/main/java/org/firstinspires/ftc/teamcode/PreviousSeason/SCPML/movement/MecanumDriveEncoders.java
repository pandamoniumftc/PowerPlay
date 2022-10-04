package org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.movement;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.Encoder;
import org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.Pose2d;
import org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.Vector2d;
import org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.*;

public class MecanumDriveEncoders extends MecanumDrive {

    private Encoder leftFrontEncoder, rightFrontEncoder, leftRearEncoder, rightRearEncoder;

    public double startingAngle;
    public double encoderTicsPerInch = 751.8347 / (3.77953 * PI);



    public MecanumDriveEncoders(@NonNull LinearOpMode opMode, HardwareMap hardwareMap, double startingangle) {
         super(opMode);

        leftFrontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "flm"), 1, encoderTicsPerInch);
        rightFrontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frm"), 1, encoderTicsPerInch);
        leftRearEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "blm"), 1, encoderTicsPerInch);
        rightRearEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "brm"), 1, encoderTicsPerInch);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        startingAngle = startingangle;
    }

    //Mecanum Wheel Encoder Calculations

    //Pose2d currentPosition = new Pose2d(0, 0);
    Vector2d currentBotCentricPosition = new Vector2d(0, 0 );
    public double currentAngle;
    Vector2d previousLeftFrontVector = new Vector2d(0, 0);
    Vector2d previousLeftRearVector = new Vector2d(0, 0);
    Vector2d previousRightFrontVector = new Vector2d(0, 0);
    Vector2d previousRightRearVector = new Vector2d(0, 0);
    Vector2d currentLeftFrontVector = new Vector2d(0);
    Vector2d currentRightFrontVector = new Vector2d(0);
    Vector2d currentLeftRearVector = new Vector2d(0);
    Vector2d currentRightRearVector = new Vector2d(0);

    @Override
    public void trackPosition() {
        currentAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle + startingAngle;
        Vector2d currentLeftFrontVector = new Vector2d(leftFront.getCurrentPosition());
        Vector2d currentRightFrontVector = new Vector2d(-rightFront.getCurrentPosition(), rightFront.getCurrentPosition());
        Vector2d currentLeftRearVector = new Vector2d(-leftRear.getCurrentPosition(), leftRear.getCurrentPosition());
        Vector2d currentRightRearVector = new Vector2d(rightRear.getCurrentPosition());
        Vector2d leftFrontDifference = Vector2d.sub(currentLeftFrontVector, previousLeftFrontVector);
        Vector2d leftRearDifference = Vector2d.sub(currentLeftRearVector, previousLeftRearVector);
        Vector2d rightRearDifference = Vector2d.sub(currentRightRearVector, previousRightRearVector);
        Vector2d rightFrontDifference = Vector2d.sub(currentRightFrontVector, previousRightFrontVector);
        previousLeftFrontVector = currentLeftFrontVector;
        previousRightFrontVector = currentRightFrontVector;
        previousLeftRearVector = currentLeftRearVector;
        previousRightRearVector = currentRightRearVector;
        currentBotCentricPosition = Vector2d.add(Vector2d.add(leftFrontDifference, leftRearDifference) , Vector2d.add(rightFrontDifference, rightRearDifference) );
        currentBotCentricPosition.div(4);
        currentBotCentricPosition.rotateRadians(Math.toRadians(-currentAngle));
        currentPosition.add(Vector2d.Pose2d(currentBotCentricPosition));
    }

    //Movement stuff

    @Override
    public void setPositionTo(Vector2d positionSet) {
        trackPosition();
        currentPosition = new Pose2d(positionSet);
        trackPosition();
    }

    @Override
    public void setPositionTo(Pose2d positionSet) {
        trackPosition();
        currentPosition = positionSet;
    }
}