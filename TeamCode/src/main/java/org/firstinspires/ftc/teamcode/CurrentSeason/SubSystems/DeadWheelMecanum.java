package org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Transform2D;

public class DeadWheelMecanum extends AbstractSubsystem {

    public DcMotor frm;
    public DcMotor flm;
    public DcMotor brm;
    public DcMotor blm;

    public DcMotorEx leftForwardE;
    public DcMotorEx rightForwardE;

    public DcMotorEx backPerpendicularE;

    public double encoderResolution = 4096;

    public double wheelRadius;
    private double wheelCircumference;

    public double Kh = 0;

    public Transform2D transform;

    public DeadWheelMecanum(AbstractRobot robot, String frmC, String flmC, String brmC, String blmC, String leftEC, String rightEC, String backEC, double resolution, double radius, double headingConstant) {
        super(robot);

        frm = robot.hardwareMap.dcMotor.get(frmC);
        flm = robot.hardwareMap.dcMotor.get(flmC);
        brm = robot.hardwareMap.dcMotor.get(brmC);
        blm = robot.hardwareMap.dcMotor.get(blmC);

        leftForwardE = robot.hardwareMap.get(DcMotorEx.class, leftEC);
        rightForwardE = robot.hardwareMap.get(DcMotorEx.class, rightEC);
        backPerpendicularE = robot.hardwareMap.get(DcMotorEx.class, backEC);

        leftForwardE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightForwardE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backPerpendicularE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftForwardE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightForwardE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backPerpendicularE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        double x = robot.gamepad1.left_stick_x;
        double y = robot.gamepad1.left_stick_y;

        double c = robot.gamepad1.right_stick_x;

        double speedMultiply =  (1-robot.gamepad1.right_trigger) * 0.75 + 0.25;

        frm.setPower(Range.clip(y+x+c, -1, 1) * speedMultiply);
        brm.setPower(Range.clip(y-x+c, -1, 1) * speedMultiply);
        flm.setPower(Range.clip(-y+x+c, -1, 1) * speedMultiply);
        blm.setPower(Range.clip(-y-x+c, -1, 1) * speedMultiply);

        updateTransform();

        robot.telemetry.addData("x: ", transform.x);
        robot.telemetry.addData("y: ", transform.y);
        robot.telemetry.addData("heading: ", transform.heading);
        robot.telemetry.update();
    }

    @Override
    public void stop() {

    }

    public Transform2D updateTransform() {
        double x = 0.5 * (leftForwardE.getCurrentPosition() + rightForwardE.getCurrentPosition()) * (1.0/encoderResolution) * wheelCircumference;
        double y = backPerpendicularE.getCurrentPosition() * (1.0/encoderResolution) * wheelCircumference;

        double heading = (leftForwardE.getCurrentPosition() - rightForwardE.getCurrentPosition()) * (1.0/encoderResolution) * wheelCircumference * Kh;

        transform = new Transform2D(x, y, heading);

        return transform;
    }
}
