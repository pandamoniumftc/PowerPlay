package org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;

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
        double x = robot.gamepad1.left_stick_x;
        double y = robot.gamepad1.left_stick_y;

        double c = robot.gamepad1.right_stick_x;

        double speedMultiply =  (1-robot.gamepad1.right_trigger) * 0.75 + 0.25;

        double biggest = Math.max(Math.max(Math.abs(y+x+c), Math.abs(y-x+c)), Math.max(Math.abs(-y+x+c),Math.abs(-y+c+c)));

        frm.setPower(speedMultiply * (y+x+c)/biggest);
        brm.setPower(speedMultiply * (y-x+c)/biggest);
        flm.setPower(speedMultiply * (-y+x+c)/biggest);
        blm.setPower(speedMultiply * (-y-x+c)/biggest);

        /*telemetry.addData("firstAngle: ", robot.imu.getAngularOrientation().firstAngle);
        telemetry.addData("secondAngle: ", robot.imu.getAngularOrientation().secondAngle);
        telemetry.addData("thirdAngle: ", robot.imu.getAngularOrientation().thirdAngle);*/

    }

    @Override
    public void stop() {

    }
}
