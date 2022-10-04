package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.SCPML.util.Vector2d;

public class MecanumDriveSubsystem extends AbstractSubsystem {
    public DcMotor frm, brm, flm, blm;

    public MecanumDriveSubsystem(AbstractRobot robot, String frm, String brm, String flm, String blm) {
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
        double x = robot.gamepad1.left_stick_x;
        double y = robot.gamepad1.left_stick_y;

        double c = robot.gamepad1.right_stick_x;

        double speedMultiply =  (1-robot.gamepad1.right_trigger) * 0.75 + 0.25;

        frm.setPower(Range.clip(y+x+c, -1, 1) * speedMultiply);
        brm.setPower(Range.clip(y-x+c, -1, 1) * speedMultiply);
        flm.setPower(Range.clip(-y+x+c, -1, 1) * speedMultiply);
        blm.setPower(Range.clip(-y-x+c, -1, 1) * speedMultiply);

    }

    @Override
    public void stop() {

    }

    public void setMotorPowers(double x, double y, double c) {
        setMotorPowers(x, y, c, 1);
    }

    public void setMotorPowers(double x, double y, double c, double speedMultiply) {
        frm.setPower(Range.clip(y+x+c, -1, 1) * speedMultiply);
        brm.setPower(Range.clip(y-x+c, -1, 1) * speedMultiply);
        flm.setPower(Range.clip(-y+x+c, -1, 1) * speedMultiply);
        blm.setPower(Range.clip(-y-x+c, -1, 1) * speedMultiply);
    }

    public void setMotorPowers(double theta) {
        setMotorPowers(theta, 1);
    }

    public void setMotorPowers(double theta, double speedMult) {
        double x = Math.cos(theta);
        double y = Math.sin(theta);

        setMotorPowers(x, y, 0, speedMult);
    }

    public void setMotorPowersTime(double x, double y, double c, double mult, long millis) {
        long start = System.currentTimeMillis();
        long end = start + millis;
        while (System.currentTimeMillis() - end < 0) {
            setMotorPowers(x, y, c, mult);
        }

        setMotorPowers(0, 0, 0);
    }

    public void setMotorPowersTime(double x, double y, double c, long millis) {
        long start = System.currentTimeMillis();
        long end = start + millis;
        while (System.currentTimeMillis() - end < 0) {
            setMotorPowers(x, y, c);
        }

        setMotorPowers(0, 0, 0);
    }

    public void waitTime(long millis) {
        long start = System.currentTimeMillis();
        long end = start + millis;

        while (System.currentTimeMillis()-end > 0) {

        }

    }

}
