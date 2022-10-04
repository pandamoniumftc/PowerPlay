package org.firstinspires.ftc.teamcode.SCPML.movement;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.SCPML.util.*;




public class MecanumDeadWheels extends MecanumDrive {

    //Constants
    public double distanceBetweenEncoders = 20; //inches
    public double frontOffset = 0; //Distance of dead wheels from center of rotation (positive is forward, negative is back)
    public double encoderTicsPerInch = 8192 / 1.37795 * PI;

    private Encoder frontEncoder, rightEncoder, leftEncoder;


    public MecanumDeadWheels(@NonNull LinearOpMode opMode, HardwareMap hardwareMap, double startingangle) {
        super(opMode);

        //Encoder init
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"), 1, encoderTicsPerInch);
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"), 1, encoderTicsPerInch);
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"), 1, encoderTicsPerInch);

        //IMU init
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        startingAngle = startingangle;
    }

    //Dead Wheels Stuff

    Pose2d currentBotCentricPosition = new Pose2d();
    Pose2d previousBotCentricPosition = new Pose2d();
    Pose2d botCentricDifference = new Pose2d();
    public double currentAngle;

    @Override
    public void trackPosition() {
        currentAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle + startingAngle;
        currentBotCentricPosition = new Pose2d(frontEncoder.getCurrentPosition(), (rightEncoder.getCurrentPosition() + leftEncoder.getCurrentPosition()) / 2);
        botCentricDifference = Pose2d.sub(currentBotCentricPosition, previousBotCentricPosition);
        botCentricDifference.rotateRadians(Math.toRadians(-currentAngle));
        botCentricDifference.div(encoderTicsPerInch);
        currentPosition.add(botCentricDifference);
        currentPosition.heading = currentAngle;
        previousBotCentricPosition = currentBotCentricPosition;
    }

    //Movement stuff

    @Override
    public void setPositionTo(Vector2d positionSet) {
        trackPosition();
        currentPosition = new Pose2d(positionSet);
    }

    @Override
    public void setPositionTo(Pose2d positionSet) {
        trackPosition();
        currentPosition = positionSet;
    }


}
