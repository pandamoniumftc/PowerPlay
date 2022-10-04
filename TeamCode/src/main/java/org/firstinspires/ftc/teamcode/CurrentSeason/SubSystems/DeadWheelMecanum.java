package org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Transform2D;

public class DeadWheelMecanum extends AbstractSubsystem {

    public DcMotorEx leftForward;
    public DcMotorEx rightForward;

    public DcMotorEx backPerpendicular;

    public double encoderResolution = 4096;

    public double wheelRadius;
    private double wheelCircumference;

    public double Kh = 0;

    public Transform2D transform;

    public DeadWheelMecanum(AbstractRobot robot, String leftC, String rightC, String backC, double resolution, double radius, double headingConstant) {
        super(robot);

        leftForward = robot.hardwareMap.get(DcMotorEx.class, leftC);
        rightForward = robot.hardwareMap.get(DcMotorEx.class, rightC);
        backPerpendicular = robot.hardwareMap.get(DcMotorEx.class, backC);

        encoderResolution = resolution;
        wheelRadius = radius;
        wheelCircumference = Math.PI * 2 * radius;
        Kh = headingConstant;

        transform = new Transform2D();
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {

    }

    @Override
    public void stop() {

    }

    public Transform2D updateTransfrom() {
        double x = 0.5 * (leftForward.getCurrentPosition() + rightForward.getCurrentPosition()) * (1.0/encoderResolution) * wheelCircumference;
        double y = backPerpendicular.getCurrentPosition() * (1.0/encoderResolution) * wheelCircumference;

        double heading = (leftForward.getCurrentPosition() - rightForward.getCurrentPosition()) * (1.0/encoderResolution) * wheelCircumference * Kh;

        transform = new Transform2D(x, y, heading);
        return transform;
    }
}
