package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public abstract class AbstractSubsystem {

    public AbstractRobot robot;

    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;

    public AbstractSubsystem(AbstractRobot robot) {
        this.robot = robot;
        this.hardwareMap = robot.hardwareMap;
        this.telemetry = robot.telemetry;
    }

    public abstract void init();

    public abstract void start();

    public abstract void driverLoop();

    public abstract void stop();


}
