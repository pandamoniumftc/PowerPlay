package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;

public class DuckSpinnerSubsystem extends AbstractSubsystem {
    public DcMotor motor;

    double power = 0.5;

    double accel = 0.001;

    public boolean isSpinning = false;

    public DuckSpinnerSubsystem(AbstractRobot robot, String motorConfig) {
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

        if (robot.gamepad2.dpad_up || robot.gamepad2.dpad_down) {
            power = power + accel;
            isSpinning = true;
        }
        else {
            power = 0.5;
            isSpinning = false;
        }

        if (robot.gamepad2.dpad_up) motor.setPower(power);
        else if (robot.gamepad2.dpad_down) motor.setPower(-power);
        else motor.setPower(0);
    }


    @Override
    public void stop() {

    }
}
