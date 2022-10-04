package org.firstinspires.ftc.teamcode.PreviousSeason.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;

public class TurnMotorON extends AbstractSubsystem {
    DcMotor motor;

    public TurnMotorON(AbstractRobot robot, String motorPath) {
        super(robot);
        motor = hardwareMap.dcMotor.get(motorPath);
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        //motor.setPower((robot.gamepad1.a) ? 1 : 0);
        motor.setPower(1);
    }

    @Override
    public void stop() {

    }
}
