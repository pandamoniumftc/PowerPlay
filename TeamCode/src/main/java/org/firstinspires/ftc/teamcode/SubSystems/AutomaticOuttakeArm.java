package org.firstinspires.ftc.teamcode.SubSystems;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.SCPML.util.Encoder;
import org.firstinspires.ftc.teamcode.SCPML.util.PID;
import org.firstinspires.ftc.teamcode.SCPML.util.Toggle;

public class AutomaticOuttakeArm extends AbstractSubsystem {
    public CRServo yaw1, yaw2;

    public Encoder turret;

    public Servo roll1, roll2, pitch, latch;

    public Toggle arm = new Toggle(false);

    public double armAngle = 0;

    public AutomaticOuttakeArm(AbstractRobot robot, String yaw1C, String yaw2C, String roll1C, String roll2C, String pitchC, String latchC) {
        super(robot);

        yaw1 = robot.hardwareMap.crservo.get(yaw1C);
        yaw2 = robot.hardwareMap.crservo.get(yaw2C);

        roll1 = robot.hardwareMap.servo.get(roll1C);
        roll2 = robot.hardwareMap.servo.get(roll2C);

        pitch = robot.hardwareMap.servo.get(pitchC);

        latch = robot.hardwareMap.servo.get(latchC);

        //turret = new Encoder(yawEncoder, 1 / 8192);
        //this.armAngle = armAngle;
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        /*double yawPower = robot.gamepad2.right_stick_x;

        arm.update(robot.gamepad1.b);
        if(arm.state && arm.needChange) {
            latch.setPosition(0.5);
            roll1.setPosition(0.5);
            roll2.setPosition(0.5);

            //moveYawTo(0);

            roll1.setPosition(0.1);
            roll2.setPosition(0.9);

            pitch.setPosition(0.65);

            arm.needChange = false;
        }
        else if (!arm.state && arm.needChange){
            latch.setPosition(1);
            sleep(500);
            latch.setPosition(0);

            pitch.setPosition(0.1);

            roll1.setPosition(0.5);
            roll2.setPosition(0.5);

            moveYawTo(armAngle);

            roll1.setPosition(0.9);
            roll2.setPosition(0.1);

            latch.setPosition(0.35);

            arm.needChange = false;
        }*/


    }

    @Override
    public void stop() {

    }

    public void moveYawTo(double angle) {
        PID armPid = new PID(PID.Type.arm);
        while(turret.getDegrees() != angle) {
            armPid.pid(turret.getDegrees() - angle);
            //yaw1.setPower(armPid.pidout);
            //yaw2.setPower(armPid.pidout);
        }
        yaw1.setPower(0);
        yaw2.setPower(0);
    }
}
