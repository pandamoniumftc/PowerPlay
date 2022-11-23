package org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;

public class NoLocalizationMecanum extends AbstractSubsystem {
    public DcMotor frm, brm, flm, blm;

    public NoLocalizationMecanum(AbstractRobot robot, String frm, String brm, String flm, String blm) {
        super(robot);

       this.frm = robot.hardwareMap.dcMotor.get(frm);
        this.brm = robot.hardwareMap.dcMotor.get(brm);
        this.flm = robot.hardwareMap.dcMotor.get(flm);
        this.blm = robot.hardwareMap.dcMotor.get(blm);
        /*
        this.frm.setDirection(DcMotorSimple.Direction.REVERSE);
        this.brm.setDirection(DcMotorSimple.Direction.REVERSE);
        this.flm.setDirection(DcMotorSimple.Direction.FORWARD);
        this.blm.setDirection(DcMotorSimple.Direction.FORWARD);
         */
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        double x = -robot.gamepad1.left_stick_x * 1.1;
        double y = robot.gamepad1.left_stick_y;

        double c = -robot.gamepad1.right_stick_x;

        double speedMultiply =  (1-robot.gamepad1.right_trigger) * 0.75 + 0.25;

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

        if (Math.abs(angle - PI) < 5 || Math.abs(angle + PI) < 5) {
            y = 0;
            x = -magnitude;
        }

        if (Math.abs(angle + PI / 2) < 5) {
            y = -magnitude;
            x = 0;
        }*/

        setMotorPower(x, y, c, speedMultiply);

        //telemetry.addData("anglea", angle);
        //telemetry.update();

        /*telemetry.addData("firstAngle: ", robot.imu.getAngularOrientation().firstAngle);
        telemetry.addData("secondAngle: ", robot.imu.getAngularOrientation().secondAngle);
        telemetry.addData("thirdAngle: ", robot.imu.getAngularOrientation().thirdAngle);*/

        /*telemetry.addData("frm", frm.getCurrentPosition());
        telemetry.addData("flm", flm.getCurrentPosition());
        telemetry.addData("brm", brm.getCurrentPosition());
        telemetry.addData("blm", blm.getCurrentPosition());
        telemetry.update();*/

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

    @Override
    public void stop() {

    }
}
