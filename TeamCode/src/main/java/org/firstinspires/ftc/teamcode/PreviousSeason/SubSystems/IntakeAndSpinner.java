package org.firstinspires.ftc.teamcode.PreviousSeason.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.PreviousSeason.Robots.SetName;

public class IntakeAndSpinner extends AbstractSubsystem {
    public DcMotor motor;
    SetName robot;
    public IntakeAndSpinner(SetName robot, String motorConfig) {
        super(robot);
        this.robot = robot;
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
        double power = robot.gamepad2.left_stick_y;

        if (robot.gamepad2.dpad_up) power = 0.5;
        if (robot.gamepad2.dpad_down) power = -0.5;
        motor.setPower(power);
    }

    @Override
    public void stop() {

    }
}
