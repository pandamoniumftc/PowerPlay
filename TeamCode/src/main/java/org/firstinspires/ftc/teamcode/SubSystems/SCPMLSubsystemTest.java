package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.SCPML.movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.SCPML.movement.MecanumDriveEncoders;
import org.firstinspires.ftc.teamcode.SCPML.util.Vector2d;

public class SCPMLSubsystemTest extends AbstractSubsystem {
    MecanumDriveEncoders drive;
    public SCPMLSubsystemTest(AbstractRobot robot, LinearOpMode opMode) {
        super(robot);
        drive = new MecanumDriveEncoders(opMode,  this.robot.hardwareMap,0);
    }



    @Override
    public void init() {

    }

    @Override
    public void start() {
        drive.lineTo(new Vector2d(0, 100));
    }

    @Override
    public void driverLoop() {

    }

    @Override
    public void stop() {

    }
}
