package org.firstinspires.ftc.teamcode.CurrentSeason.Util;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;

public class PIDController {
    public double Kp, Ki, Kd;

    public double margin = 0.2;

    private double target;
    private double angle;
    private double error;

    private double integral;
    private double prevError;

    private long prevTime;

    private boolean firstFrame = true;

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
    }

    public PIDController(double Kp, double Ki, double Kd, double margin) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;

        this.margin = margin;
    }

    public double PIDOutput(double target, double current) throws Exception {
        if (target < -Math.PI/2 || target > Math.PI/2) throw new Exception("target angle must be between positive and negative 2PI");

        angle = current;
        error = target - angle;

        if (error < margin) return 0;

        double dt = (firstFrame) ? 0 : (System.nanoTime() - prevTime) * 1E-9;

        integral += (error - prevError) * dt;

        double P = Kp * error;
        double I = Ki * integral;
        double D = (firstFrame) ? 0 : Kd * (error - prevError) / dt;

        double PID = P + I + D;

        prevError = error;
        prevTime = System.nanoTime();

        firstFrame = false;

        return PID;
    }
}