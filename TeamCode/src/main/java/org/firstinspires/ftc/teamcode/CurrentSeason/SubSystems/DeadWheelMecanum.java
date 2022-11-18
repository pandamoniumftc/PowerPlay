package org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems;

import static java.lang.Math.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.PIDController;
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
        wheelCircumference = PI * 2 * radius;
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

        double max = max(max(abs(y+x+c), abs(y-x+c)), max(abs(-y+x+c), abs(-y-x+c)));
        double coefficient = speedMultiply/max;

        updateTransform();

        robot.telemetry.addData("x: ", transform.x);
        robot.telemetry.addData("y: ", transform.y);
        robot.telemetry.addData("heading: ", transform.heading);
        robot.telemetry.update();
    }

    @Override
    public void stop() {

    }

    public void setMotorPower(double x, double y, double c, double speedMultiply) {
        double max = max(max(abs(y+x+c), abs(y-x+c)), max(abs(-y+x+c), abs(-y-x+c)));
        if (max == 0) {
            setMotorPower(0, 1, 0, 0);
            return;
        }

        double coefficient = speedMultiply/max;

        frm.setPower( ( y+x+c) * coefficient);
        brm.setPower( ( y-x+c) * coefficient);
        flm.setPower( (-y+x+c) * coefficient);
        blm.setPower( (-y-x+c) * coefficient);
    }

    public void setMotorPower(double r, double theta) {
        double x = r * cos(theta);
        double y = r * sin(theta);

        double max = max(max(abs(y+x), abs(y-x)), max(abs(-y+x), abs(-y-x)));
        double coefficient = 1/max;

        frm.setPower( ( y+x) * coefficient);
        brm.setPower( ( y-x) * coefficient);
        flm.setPower( (-y+x) * coefficient);
        blm.setPower( (-y-x) * coefficient);
    }

    public void driveForwardEncoders(double inches) throws Exception {
        updateTransform();
        double x0 = transform.x;
        double y0 = transform.y;
        double h0 = transform.heading;

        double x1 = transform.x + inches * cos(h0);
        double y1 = transform.y + inches * sin(h0);
        double h1 = h0;

        double headingTarget = h1 - h0;
        double translationalTarget = sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));

        double headingError = headingTarget;
        double translationalError = translationalTarget;

        PIDController headingController = new PIDController(1, 0, 0.3, 0.05);
        PIDController translationalController = new PIDController(1, 0, 0.3, 0.2);

        while (headingError > headingController.margin && translationalError > translationalController.margin) {
            updateTransform();

            double y = translationalController.PIDOutput(translationalTarget, sqrt((transform.x - x0) * (transform.x - x0) + (transform.y - y0) * (transform.y - y0)));
            double c = headingController.PIDOutput(headingTarget, transform.heading - h0);

            translationalError = translationalTarget - sqrt((transform.x - x0) * (transform.x - x0) + (transform.y - y0) * (transform.y - y0));
            headingError = h1 - transform.heading;

            setMotorPower(0, y, c, 0.5);
        }
        setMotorPower(0, 0, 0, 0);
    }

    public Transform2D updateTransform() {
        double x = 0.5 * (leftForwardE.getCurrentPosition() + rightForwardE.getCurrentPosition()) * (1.0/encoderResolution) * wheelCircumference;
        double y = backPerpendicularE.getCurrentPosition() * (1.0/encoderResolution) * wheelCircumference;

        double heading = (leftForwardE.getCurrentPosition() - rightForwardE.getCurrentPosition()) * (1.0/encoderResolution) * wheelCircumference * Kh;

        transform = new Transform2D(x, y, heading);

        return transform;
    }
}
