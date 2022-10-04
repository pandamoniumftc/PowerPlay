package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.Robots.SetName;
import org.firstinspires.ftc.teamcode.SCPML.movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.SCPML.movement.MecanumDriveEncoders;
import org.firstinspires.ftc.teamcode.SCPML.util.NanoClock;
import org.firstinspires.ftc.teamcode.SCPML.util.Vector2d;

@Autonomous(name="SCPML autoTest")
public class SCPMLtest extends AbstractAutonomous {

    //HardwareMap hardwareMap = new HardwareMap();

    MecanumDriveEncoders drive;

    @Override
    public void onInit() {
        drive = new MecanumDriveEncoders(this, robot.hardwareMap, 0);
    }

    @Override
    public void autonomous() {
        SetName robot = (SetName) getRobot();

        NanoClock clock = new NanoClock();

        drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);

        while(clock.secondsLifeSpan() < 2) {
            drive.trackPosition();
            telemetry.addData("Current x", drive.currentPosition.X);
            telemetry.addData("Current y", drive.currentPosition.Y);
            telemetry.addData("Current Angle", drive.currentAngle);
            telemetry.update();
        }

    }

    @Override
    public AbstractRobot instantiateRobot() {
        return new SetName(this);

    }


}