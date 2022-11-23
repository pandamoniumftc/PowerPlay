package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.RizzBot;

@Autonomous(name="encoder test")
public class EncoderTestAuto extends AbstractAutonomous {
    public RizzBot robot;

    @Override
    public AbstractRobot instantiateRobot() {
        robot = new RizzBot(this);
        return robot;
    }

    @Override
    public void autonomous() {
        long time = System.nanoTime();
        driveForTime(1000, 1, 0, 0, 0.3);
        driveForTime(10000000, 0, 0, 0, 0);
    }

    private void driveForTime(long millis, double x, double y, double c, double speedMultiply) {
        long time = System.currentTimeMillis() + millis;

        while (System.currentTimeMillis() < time) {
            robot.drive.setMotorPower(x, y, c, speedMultiply);
            robot.drive.updateTransform();
            telemetry.addData("x: ", robot.drive.t.x);
            telemetry.addData("y: ", robot.drive.t.y);
            telemetry.addData("h: ", robot.drive.t.heading);
            telemetry.update();
        }
    }



}
