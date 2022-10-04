package org.firstinspires.ftc.teamcode.PreviousSeason.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;

public class IntakeArm extends AbstractSubsystem {
    public DcMotor motor;

    public IntakeArm(AbstractRobot robot, String motorConfig) {
        super(robot);

        motor = robot.hardwareMap.dcMotor.get(motorConfig);
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        double power = 0;

        if (robot.gamepad1.a) power++;
        if (robot.gamepad1.b) power--;

        motor.setPower(power);
    }

    @Override
    public void stop() {

    }
}
