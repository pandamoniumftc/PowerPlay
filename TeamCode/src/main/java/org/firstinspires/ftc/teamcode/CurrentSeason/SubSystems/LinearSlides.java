package org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;

public class LinearSlides extends AbstractSubsystem {
    public DcMotor motor;

    public DcMotor leftVertical;
    public DcMotor rightVertical;

    public DcMotor horExtension;

    public double avg_height;
    public int max_position;
    public int min_position;

    public double encoderResolution;

    public LinearSlides(AbstractRobot robot, String motorConfig, String leftVertEC, String rightVertEC, String extEC, int max, int min, double resolution, double gearRatio) {
        super(robot);

        this.robot = robot;

        motor = robot.hardwareMap.dcMotor.get(motorConfig);

        leftVertical = robot.hardwareMap.get(DcMotorEx.class, leftVertEC);
        rightVertical = robot.hardwareMap.get(DcMotorEx.class, rightVertEC);
        horExtension = robot.hardwareMap.get(DcMotorEx.class, extEC);

        leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encoderResolution = resolution;

        double encoderToInchRatio = gearRatio * 1/(encoderResolution);

        avg_height = 0.5 * (leftVertical.getCurrentPosition() + rightVertical.getCurrentPosition()) * encoderToInchRatio;

        max_position = max;

        min_position = min;

    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {

        double power = robot.gamepad2.left_stick_y;

        double speedMultiplier = ((1-robot.gamepad1.left_trigger) * 0.75 + 0.25);

        if (avg_height >= max_position) {

            power = Range.clip(power, -1, 0);

        }

        if (avg_height <= min_position) {

            power = Range.clip(power, 0, 1);

        }

        leftVertical.setPower(power*speedMultiplier);
        rightVertical.setPower(power*speedMultiplier);

        telemetry.addData("Average Position: ", avg_height);

    }

    @Override
    public void stop() {

    }

}