package org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Pulse;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;

public class LinearSlides extends AbstractSubsystem {
    public DcMotor motor;

    public DcMotor rightVertical;

    public DcMotor horExtension;

    public Servo clamp;

    public int max_position;
    public int min_position;

    public double encoderResolution;
    public double encoderToInchRatio;

    public Toggle servoState = new Toggle(false);


    public int[] slidePositions = new int[] {0, 1000, 2000, 3000};
    public int slideLevel = 0;

    public Pulse upPulse = new Pulse();
    public Pulse downPulse = new Pulse();

    public LinearSlides(AbstractRobot robot, String rightVertEC, String extEC, String clampConfig, int max, int min, double resolution, double gearRatio) {
        super(robot);

        this.robot = robot;

        rightVertical = robot.hardwareMap.get(DcMotorEx.class, rightVertEC);
        horExtension = robot.hardwareMap.get(DcMotorEx.class, extEC);

        rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clamp = robot.hardwareMap.servo.get(clampConfig);

        encoderResolution = resolution;
        encoderToInchRatio = gearRatio * 1/(encoderResolution);

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

        double vPower = robot.gamepad2.left_stick_y;
        double hPower = -robot.gamepad2.right_stick_x;

        double speedMultiplier = ((1-robot.gamepad1.left_trigger) * 0.75 + 0.25);

        //double disparity = (leftVertical.getCurrentPosition() - rightVertical.getCurrentPosition()) * 0.001;

        if (vPower != 0) {
            rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        rightVertical.setPower(vPower*speedMultiplier);

        boolean up = upPulse.update(robot.gamepad2.dpad_up);
        boolean down = downPulse.update(robot.gamepad2.dpad_down);

        if (up || down) {
            rightVertical.setPower(0.75);

            slideLevel += (up) ? 1 : 0;
            slideLevel -= (down) ? 1 : 0;

            slideLevel = Range.clip(slideLevel, 0, slidePositions.length);

            rightVertical.setTargetPosition(slidePositions[slideLevel]);

            rightVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        horExtension.setPower(hPower*speedMultiplier);

        //telemetry.addData("Average Position: ", avg_height);
        //telemetry.update();

        double servoPos = servoState.updateState(robot.gamepad2.a) ? 1.0 : 0;
        clamp.setPosition(servoPos);


    }

    @Override
    public void stop() {

    }

}
