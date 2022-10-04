package org.firstinspires.ftc.teamcode.PreviousSeason.SubSystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.Encoder;
import org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.PID;
import org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util.Toggle;

public class ColorSensorOuttakeArmIntake extends AbstractSubsystem {
    public CRServo yaw1, yaw2;

    public RevColorSensorV3 colorSensor;

    public Encoder turretEncoder;

    public DcMotor intake;

    public Servo roll1, roll2, pitch, latch;

    public Toggle arm = new Toggle(false);
    public Toggle upT = new Toggle(false);
    public Toggle downT = new Toggle(false);
    public Toggle rollT = new Toggle(false);

    public Toggle pitchT = new Toggle(true);
    public Toggle latchT = new Toggle(false);


    public double armAngle = 0;

    PID armPid = new PID(PID.Type.arm);
    boolean isTurning = false;
    boolean isArmExtended = false;
    boolean isArmExtending = false;
    boolean isArmExtendingBack = false;
    boolean prevArmToggleState = false;

    public long pitchMoveTimeStamp = 0;
    public boolean pitchMoveQueued = false;

    public ColorSensorOuttakeArmIntake(AbstractRobot robot, String yaw1C, String yaw2C, String roll1C, String roll2C, String pitchC, String latchC, String colorSensor, String intake, String yawEncoder,  double armAngle) {
        super(robot);

        this.colorSensor = robot.hardwareMap.get(RevColorSensorV3.class, colorSensor);
        turretEncoder = new Encoder(robot.hardwareMap.get(DcMotorEx.class, "turret_encoder"), 360.0/8192);
        robot.hardwareMap.get(DcMotorEx.class, yawEncoder).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hardwareMap.get(DcMotorEx.class, yawEncoder).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yaw1 = robot.hardwareMap.crservo.get(yaw1C);
        yaw2 = robot.hardwareMap.crservo.get(yaw2C);

        roll1 = robot.hardwareMap.servo.get(roll1C);
        roll2 = robot.hardwareMap.servo.get(roll2C);

        pitch = robot.hardwareMap.servo.get(pitchC);

        latch = robot.hardwareMap.servo.get(latchC);

        this.intake = robot.hardwareMap.dcMotor.get(intake);

        this.armAngle = armAngle;


    }

    @Override
    public void init() {
         //latch.setPosition(0.5);
    }

    @Override
    public void start() {
        //latch.setPosition(0.5);
    }

    @Override
    public void driverLoop() {
        double yawPower = -robot.gamepad2.right_stick_x;

        if(robot.gamepad2.right_bumper && !isTurning) {
            armPid.reset();
            isTurning = true;
        }

        if ((Math.abs(turretEncoder.getDegrees() - targetAngle) > 1) && isTurning) {
            moveYawTo(0);
        }
        else {
            isTurning = false;
        }

        /*arm.update(robot.gamepad1.x);
        if(arm.state && arm.needChange) {
            intake.setPower(1);
        } else if(colorSensor.alpha() == 0) {
            intake.setPower(0);
            latch.setPosition(0);
            roll1.setPosition(0.5);
            roll2.setPosition(0.5);

            moveYawTo(0);

            roll1.setPosition(1);
            roll2.setPosition(1);

            pitch.setPosition(0);
        } else if (!arm.state && arm.needChange){
            latch.setPosition(1);
            sleep(500);
            latch.setPosition(0);

            pitch.setPosition(1);

            roll1.setPosition(0.5);
            roll2.setPosition(0.5);

            moveYawTo(armAngle);

            roll1.setPosition(0);
            roll2.setPosition(0);

            latch.setPosition(1);

            arm.needChange = false;
        }
         */
        if (!isTurning || yawPower != 0) {
            yaw1.setPower(yawPower);
            yaw2.setPower(yawPower);

            isTurning = false;
        }

        /*
        upT.update(robot.gamepad2.dpad_up);
        downT.update(robot.gamepad2.dpad_down);
        rollState += (upT.state ? 1 : 0) - (downT.state ? 1 : 0);
        rollState = Range.clip(rollState, 0, 1);
        */

        if (!isArmExtending && (turretEncoder.getDegrees() > -4 && turretEncoder.getDegrees() < 4))
            rollT.update(robot.gamepad2.a);
        if (rollT.state != prevArmToggleState) {
            isArmExtending = true;
            isArmExtendingBack = isArmExtended;

            pitchMoveTimeStamp = System.currentTimeMillis();

            isArmExtended = !isArmExtended;
        }

        if (!isArmExtending) {
            pitchT.update(robot.gamepad2.x);
            pitch.setPosition((pitchT.state) ? 0.65 : 0.35);
        }
        if (isArmExtending) {
            if (isArmExtendingBack) {
                pitchMoveQueued = true;

                pitch.setPosition(0.65);
                pitchT.state = true;
                if (System.currentTimeMillis()-pitchMoveTimeStamp > 600)
                {
                    roll1.setPosition((rollT.state) ? 0.7 : 0.1);
                    roll2.setPosition((rollT.state) ? 0.3 : 0.9);
                }
                if (System.currentTimeMillis()-pitchMoveTimeStamp > 1600) {
                    isArmExtending = false;

                    latchT.state = false;
                    latch.setPosition(0.5);
                }
            }

            else {
                roll1.setPosition((rollT.state) ? 0.9 : 0);
                roll2.setPosition((rollT.state) ? 0.1 : 1);
 
                if (System.currentTimeMillis()-pitchMoveTimeStamp > 1500)
                {
                    pitch.setPosition(0.35);
                    pitchT.state = false;
                    isArmExtending = false;
                }
            }
        }


        if (!isArmExtending) {
            latchT.update(robot.gamepad2.y);
            latch.setPosition(0.35);
        }
        if (robot.gamepad2.a) latchT.state = true;
        latch.setPosition((latchT.state) ? 0.35 : 0.5);


        telemetry.addData("Degrees", turretEncoder.getDegrees());
        telemetry.addData("color sensor alpha", colorSensor.alpha());
        telemetry.addData("color sensor dist", colorSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();


        prevArmToggleState = rollT.state;
    }

    @Override
    public void stop() {

    }

    double targetAngle;

    public void moveYawTo(double angle) {

        armPid.pid(turretEncoder.getDegrees() - angle);
        telemetry.addData("PID", armPid.pidout);
        telemetry.addData("error", turretEncoder.getDegrees()-angle);
        telemetry.update();
        yaw1.setPower(Range.clip(armPid.pidout, -1, 1));
        yaw2.setPower(Range.clip(-armPid.pidout, -1, 1));
        targetAngle = angle;
    }

}
