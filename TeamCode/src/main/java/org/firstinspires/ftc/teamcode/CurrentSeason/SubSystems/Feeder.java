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
    public Servo claw, lFlip1, rFlip1, lFlip2, rFlip2;

    public Toggle flip1Toggle;
    public Toggle flip2Toggle;
    public Toggle clawToggle;
    public Toggle extendToggle;


    public Feeder(AbstractRobot robot, String extendC, String clawC, String lFlip1C, String rFlip1C, String lFlip2C, String rFlip2C) {
        super(robot);

        extend = robot.hardwareMap.dcMotor.get(extendC);

        claw = robot.hardwareMap.servo.get(clawC);
        lFlip1 = robot.hardwareMap.servo.get(lFlip1C);
        rFlip1 = robot.hardwareMap.servo.get(rFlip1C);
        lFlip2 = robot.hardwareMap.servo.get(lFlip2C);
        rFlip2 = robot.hardwareMap.servo.get(rFlip2C);

        flip1Toggle = new Toggle(false);
        flip2Toggle = new Toggle(false);
        clawToggle = new Toggle(false);
        extendToggle = new Toggle(false);

    }

    @Override
    public void init() {
        /*claw.setPosition(0);
        lFlip1.setPosition(0);
        rFlip1.setPosition(0);
        lFlip2.setPosition(0);
        rFlip2.setPosition(0);*/
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        flip1Toggle.updateState(robot.gamepad2.x);
        flip2Toggle.updateState(robot.gamepad2.y);

        clawToggle.updateState(robot.gamepad2.a);

        claw.setPosition( (clawToggle.state) ? 1 : 0 );
        lFlip1.setPosition( (flip1Toggle.state) ? 1 : 0 );
        rFlip1.setPosition( (flip1Toggle.state) ? 1 : 0 );
        lFlip2.setPosition( (flip2Toggle.state) ? 1 : 0 );
        rFlip2.setPosition( (flip2Toggle.state) ? 1 : 0 );

        if (robot.gamepad2.b) {
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
