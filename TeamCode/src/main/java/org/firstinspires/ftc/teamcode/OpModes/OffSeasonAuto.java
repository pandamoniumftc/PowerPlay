package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.Robots.SetName;
import org.firstinspires.ftc.teamcode.SCPML.movement.MecanumDriveEncoders;
import org.firstinspires.ftc.teamcode.SCPML.util.Vector2d;

@Autonomous(name="auto blue")
public class OffSeasonAuto extends AbstractAutonomous {
    @Override
    public void autonomous() {
        SetName robot = (SetName) getRobot();

        robot.drive.setMotorPowersTime(0, 0.5, 0, 1000);

        robot.intake.motor.setPower(0.5);
        sleep(4000);

        robot.intake.motor.setPower(0);
        robot.drive.setMotorPowersTime(0, -0.5, 0, 2000);

    }

    @Override
    public AbstractRobot instantiateRobot() {
        return new SetName(this);

    }


}
