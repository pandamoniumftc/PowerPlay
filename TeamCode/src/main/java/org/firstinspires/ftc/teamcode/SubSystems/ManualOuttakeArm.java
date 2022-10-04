package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.SCPML.util.Toggle;

public class ManualOuttakeArm extends AbstractSubsystem {
    public CRServo yaw1, yaw2;

    public Servo roll1, roll2, pitch, latch;

    public Toggle upT = new Toggle(false);
    public Toggle downT = new Toggle(false);
    public Toggle rollT = new Toggle(false);

    public Toggle pitchT = new Toggle(true);
    public Toggle latchT = new Toggle(true);

    public double rollState = 0;

    public ManualOuttakeArm(AbstractRobot robot, String yaw1C, String yaw2C, String roll1C, String roll2C, String pitchC, String latchC) {
        super(robot);

        yaw1 = robot.hardwareMap.crservo.get(yaw1C);
        yaw2 = robot.hardwareMap.crservo.get(yaw2C);

        roll1 = robot.hardwareMap.servo.get(roll1C);
        roll2 = robot.hardwareMap.servo.get(roll2C);

        pitch = robot.hardwareMap.servo.get(pitchC);

        latch = robot.hardwareMap.servo.get(latchC);
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        double yawPower = robot.gamepad2.right_stick_x;

        yaw1.setPower(yawPower);
        yaw2.setPower(yawPower);

        /*
        upT.update(robot.gamepad2.dpad_up);
        downT.update(robot.gamepad2.dpad_down);
        rollState += (upT.state ? 1 : 0) - (downT.state ? 1 : 0);
        rollState = Range.clip(rollState, 0, 1);
        */

        rollT.update(robot.gamepad2.dpad_up);
        roll1.setPosition((rollT.state) ? 0.85 : 0.1);
        roll2.setPosition((rollT.state) ? 0.15 : 0.9);

        pitchT.update(robot.gamepad2.x);
        pitch.setPosition((pitchT.state) ? 0.65 : 0.1);

        latchT.update(robot.gamepad2.y);
        latch.setPosition((latchT.state) ? 0.35 : 0.5);
    }

    @Override
    public void stop() {

    }
}
