package org.firstinspires.ftc.teamcode.PreviousSeason.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.Toggle;

public class TurnServoOn extends AbstractSubsystem {
    Servo servo;
    Toggle toggle = new Toggle(false);

    public TurnServoOn(AbstractRobot robot, String crServoPath) {
        super(robot);
        servo = hardwareMap.servo.get(crServoPath);

    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        toggle.update(robot.gamepad1.a);
        telemetry.addData("A: ", robot.gamepad1.a);
        telemetry.addData("held last frame: ", toggle.heldLastFrame);
        telemetry.addData("state", toggle.state);
        telemetry.update();
        servo.setPosition((toggle.state) ? 1 : 0.5);
    }

    @Override
    public void stop() {

    }
}
