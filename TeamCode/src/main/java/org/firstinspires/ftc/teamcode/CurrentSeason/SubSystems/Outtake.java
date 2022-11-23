package org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;

public class Outtake extends AbstractSubsystem {

    public DcMotor slideMotor;
    public Servo rotateServo, clawServo, flipServo;
    public Toggle rotateToggle, clawToggle, flipToggle;

    public Outtake(AbstractRobot robot, String slideM, String rotateS, String clawS, String flipS) {
        super(robot);

        slideMotor = robot.hardwareMap.dcMotor.get(slideM);
        rotateServo = robot.hardwareMap.servo.get(rotateS);
        clawServo = robot.hardwareMap.servo.get(clawS);
        flipServo = robot.hardwareMap.servo.get(flipS);

        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotateToggle = new Toggle(false);
        clawToggle = new Toggle(false);
        flipToggle = new Toggle(false);

    }

    @Override
    public void init() {
        rotateServo.setPosition(0);
        clawServo.setPosition(0);
        flipServo.setPosition(0);
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        rotateToggle.updateState(robot.gamepad2.dpad_left);
        clawToggle.updateState(robot.gamepad2.dpad_right);
        flipToggle.updateState(robot.gamepad2.dpad_up);

        double power = robot.gamepad2.right_stick_y;

        slideMotor.setPower(power);

        rotateServo.setPosition( (rotateToggle.state) ? 1 : 0);
        clawServo.setPosition( (clawToggle.state) ? 1 : 0);
        flipServo.setPosition( (flipToggle.state) ? 1 : 0);

    }

    @Override
    public void stop() {

    }
}
