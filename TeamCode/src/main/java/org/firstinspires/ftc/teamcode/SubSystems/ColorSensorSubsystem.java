package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;

public class ColorSensorSubsystem extends AbstractSubsystem {
    ColorSensor sensor;
    double lastLoopTime;
    double loopTime;

    public ColorSensorSubsystem(AbstractRobot robot, String color_config) {
        super(robot);
        sensor = hardwareMap.colorSensor.get(color_config);

    }

    @Override
    public void init() {
        telemetry.addData("Color Subsystem", "Initializing...");
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        loopTime = System.nanoTime() * 10e-7 - lastLoopTime;
        lastLoopTime = loopTime;
        telemetry.addData("loopTime", loopTime);
        telemetry.addData("connection info", sensor.getConnectionInfo());
        telemetry.addData("alpha", sensor.alpha());
        telemetry.addData("red", sensor.red());

        //telemetry.addData("translated", 1023 * sensor.getVoltage()/3.3);
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}
