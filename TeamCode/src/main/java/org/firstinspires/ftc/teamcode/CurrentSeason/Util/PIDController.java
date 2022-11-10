package org.firstinspires.ftc.teamcode.CurrentSeason.Util;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;

public class PIDController {
    public double Kp, Ki, Kd;

    public double target, error;

    private double sum;

    private long prevTime, dt;

    private AbstractRobot bot;

    public PIDController(double Kp, double Ki, double Kd, AbstractRobot bot) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        this.bot = bot;

        this.sum = 0;
    }
    
    public void TurnPID(double target) {
        this.target = target;
        
        //error = bot.imu.getAngularOrientation().
    }
}
