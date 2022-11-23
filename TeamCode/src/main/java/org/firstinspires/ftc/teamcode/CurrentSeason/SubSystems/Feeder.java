package org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;

public class Feeder extends AbstractSubsystem {
    public DcMotor extend;
    public Servo claw, lFlip, rFlip;

    public Toggle flipToggle;
    public Toggle clawToggle;
    public Toggle extendToggle;


    public Feeder(AbstractRobot robot, String extendC, String clawC, String lFlipC, String rFlipC) {
        super(robot);

        extend = robot.hardwareMap.dcMotor.get(extendC);

        claw = robot.hardwareMap.servo.get(clawC);
        lFlip = robot.hardwareMap.servo.get(lFlipC);
        rFlip = robot.hardwareMap.servo.get(rFlipC);

        flipToggle = new Toggle(false);
        clawToggle = new Toggle(false);
        extendToggle = new Toggle(false);

    }

    @Override
    public void init() {
        claw.setPosition(0);
        lFlip.setPosition(0);
        rFlip.setPosition(0);
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        flipToggle.updateState(robot.gamepad2.x);
        clawToggle.updateState(robot.gamepad2.a);

        claw.setPosition( (clawToggle.state) ? 1 : 0 );
        lFlip.setPosition( (flipToggle.state) ? 1 : 0 );
        rFlip.setPosition( (flipToggle.state) ? 1 : 0 );

        if (robot.gamepad2.y) {
            boolean prevState = extendToggle.state;
            extendToggle.updateState(true);

            if (extendToggle.state != prevState)  {

                extend.setTargetPosition( (extendToggle.state) ? 1000 : 0);
                extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extend.setPower(0.75);
                while (Math.abs(extend.getCurrentPosition() - ((extendToggle.state) ? 1000 : 0)) > 10) {
                    try {
                        wait(1);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
            else {
                double extendPower = robot.gamepad2.left_stick_x;
                extend.setPower(extendPower);
            }
        }
        else {
            extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extendToggle.updateState(false);

            double extendPower = robot.gamepad2.left_stick_x;
            extend.setPower(extendPower);
        }
    }

    @Override
    public void stop() {

    }
}
