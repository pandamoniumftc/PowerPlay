package org.firstinspires.ftc.teamcode.SCPML.util;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder extends Object {

    private double velocityEstimate;
    private final double timeLastUpdate;
    private DcMotorEx motor;
    private double lastPosition;
    private Direction direction;
    private double gearRatio = 1;
    private double encoderTicsPerInch = 1;
    private NanoClock clock = new NanoClock();

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    private double getMultiplier() {
        return gearRatio * this.direction.multiplier / encoderTicsPerInch;
    }

    public Encoder(DcMotorEx motor, double GearRatio, double EncoderTicsPerInch){
        this.motor = motor;

        this.direction = Direction.FORWARD;
        this.lastPosition = 0;
        this.velocityEstimate = 0;
        this.timeLastUpdate = 0;
        gearRatio = GearRatio;
        encoderTicsPerInch = EncoderTicsPerInch;
    }

    public Encoder(DcMotorEx motor, double GearRatio){
        this.motor = motor;

        this.direction = Direction.FORWARD;
        this.lastPosition = 0;
        this.velocityEstimate = 0;
        this.timeLastUpdate = 0;
        gearRatio = GearRatio;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public void setGearRatio(int gearratio) {
        this.gearRatio = gearratio;
    }

    public Direction getDirection() {
        return direction;
    }

    private double lastUpdateTime;

    public double getCurrentPosition() {
        double multiplier = getMultiplier();
        double currentPosition = motor.getCurrentPosition() * multiplier;
        if (currentPosition != lastPosition) {
            double currentTime = clock.nanoLifespan();
            double dt = currentTime - lastUpdateTime;
            velocityEstimate = (currentPosition - lastPosition) / dt;
            lastPosition = currentPosition;
            lastUpdateTime = currentTime;
        }
        return currentPosition;
    }

    public double getDegrees() {
        double multiplier = gearRatio;
        double currentPosition = motor.getCurrentPosition() * multiplier;
        if (currentPosition != lastPosition) {
            double currentTime = clock.nanoLifespan();
            double dt = currentTime - lastUpdateTime;
            velocityEstimate = (currentPosition - lastPosition) / dt;
            lastPosition = currentPosition;
            lastUpdateTime = currentTime;
        }
        return currentPosition;
    }

    public void reset(){
        motor.setMode(STOP_AND_RESET_ENCODER);
    }
}