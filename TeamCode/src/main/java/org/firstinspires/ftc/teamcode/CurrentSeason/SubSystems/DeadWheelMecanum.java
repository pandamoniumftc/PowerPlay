package org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems;

import static java.lang.Math.*;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
    
    public BHI260IMU imu;

    public DcMotorEx backPerpendicularE;

    public double trackWidth;
    public double forwardOffset;
    public double inchPerTick;

    public Transform2D t0;
    public Transform2D t;

    private double prevL;
    private double prevR;
    private double prevH;

    public DeadWheelMecanum(AbstractRobot robot, String frmC, String flmC, String brmC, String blmC, String leftEC, String rightEC, String backEC, double trackWidth, double forwardOffset, double inchPerTick, Transform2D initialPos) {
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

        leftForwardE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightForwardE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backPerpendicularE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.trackWidth = trackWidth;
        this.forwardOffset = forwardOffset;
        this.inchPerTick = inchPerTick;

        this.t0 = initialPos;
        this.t = new Transform2D(initialPos.x, initialPos.y, initialPos.heading);

        prevL = leftForwardE.getCurrentPosition();
        prevR = rightForwardE.getCurrentPosition();
        prevH = backPerpendicularE.getCurrentPosition();
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        double x = robot.gamepad1.right_stick_x;
        double y = robot.gamepad1.left_stick_y;

        double c = -robot.gamepad1.left_stick_x;

        double speedMultiply = 0.4 + (0.6 * robot.gamepad1.right_trigger);

        /*double angle = Math.atan2(y, x);
        double magnitude = Math.sqrt(y*y + x*x);

        if (Math.abs(angle) < 5) {
            y = 0;
            x = magnitude;
        }

        if (Math.abs(angle - PI / 2) < 5) {
            y = magnitude;
            x = 0;
        }

        if (Math.abs(angle - PI) < 5) {
            y = 0;
            x = -magnitude;
        }

        if (Math.abs(angle - 3 * PI / 2) < 5) {
            y = -magnitude;
            x = 0;
        }*/

        setMotorPower(x, y, c, speedMultiply);

        updateTransform();

        robot.telemetry.addData("speedMultiply: ", speedMultiply);
        robot.telemetry.addData("raw x: ", x);
        robot.telemetry.addData("raw y: ", y);
        robot.telemetry.addData("raw c", c);
        robot.telemetry.addData("x: ", t.x);
        robot.telemetry.addData("y: ", t.y);
        robot.telemetry.addData("heading: ", t.heading);
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

        robot.telemetry.addData("coefficient: ", coefficient);

    }

    /*public void setMotorPower(double r, double theta) {
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

        PIDController headingController = new PIDController(1, 0, 0, 0.05);
        PIDController translationalController = new PIDController(1, 0, 0, 0.2);

        while (headingError > headingController.margin && translationalError > translationalController.margin) {
            updateTransform();

            double y = translationalController.PIDOutput(translationalTarget, sqrt((transform.x - x0) * (transform.x - x0) + (transform.y - y0) * (transform.y - y0)));
            double c = headingController.PIDOutput(headingTarget, transform.heading - h0);

            translationalError = translationalTarget - sqrt((transform.x - x0) * (transform.x - x0) + (transform.y - y0) * (transform.y - y0));
            headingError = h1 - transform.heading;

            setMotorPower(0, y, c, 0.5);
        }
        setMotorPower(0, 0, 0, 0);
    }*/

    public Transform2D updateTransform() {
        /*
        delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos
        delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos
        delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos

        phi = (delta_left_encoder_pos - delta_right_encoder_pos) / trackwidth
        delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2
        delta_perp_pos = delta_center_encoder_pos - forward_offset * phi

        delta_x = delta_middle_pos * cos(heading) - delta_perp_pos * sin(heading)
        delta_y = delta_middle_pos * sin(heading) + delta_perp_pos * cos(heading)

        x_pos += delta_x
        y_pos += delta_y
        heading += phi

        prev_left_encoder_pos = left_encoder_pos
        prev_right_encoder_pos = right_encoder_pos
        prev_center_encoder_pos = center_encoder_pos
        */

        double dxl = (-leftForwardE.getCurrentPosition() - prevL ) * inchPerTick;
        double dxr = (rightForwardE.getCurrentPosition() - prevR) * inchPerTick;
        double dc = (-backPerpendicularE.getCurrentPosition() - prevH) * inchPerTick;

        double phi = (dxl - dxr) / trackWidth;
        double dm = (dxl + dxr) / 2;
        double dh = dc - (forwardOffset * phi);

        double dx = dm * cos(t.heading) - dh * sin(t.heading);
        double dy = dm * sin(t.heading) + dh * cos(t.heading);

        t.x += dx;
        t.y += dy;
        t.heading += phi;

        prevL = -leftForwardE.getCurrentPosition();
        prevR = rightForwardE.getCurrentPosition();
        prevH = -backPerpendicularE.getCurrentPosition();

        telemetry.addData("xl: ", -leftForwardE.getCurrentPosition());
        telemetry.addData("xr: ", rightForwardE.getCurrentPosition());
        telemetry.addData("c: ", backPerpendicularE.getCurrentPosition());
        telemetry.addData("dxl: ", dxl);
        telemetry.addData("dxr: ", dxr);
        telemetry.addData("dc: ", dc);
        telemetry.addData("phi: ", phi);

        return t;
    }
}
