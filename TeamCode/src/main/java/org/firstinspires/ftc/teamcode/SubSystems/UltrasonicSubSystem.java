package org.firstinspires.ftc.teamcode.SubSystems;

import android.hardware.Sensor;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtUltrasonicSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.SCPML.util.Ultrasonic;

public class UltrasonicSubSystem extends AbstractSubsystem {
    AnalogInput sensor;
    Ultrasonic ultrasonicSensor;
    double lastLoopTime;
    double loopTime;
    double inchesPerVolt = 0.0163347;

    public UltrasonicSubSystem(AbstractRobot robot, String ultrasonic_config) {
        super(robot);
        sensor = hardwareMap.analogInput.get("ultrasonic");
        ultrasonicSensor = new Ultrasonic(sensor);
    }

    @Override
    public void init() {
        telemetry.addData("Ultrasonic Subsystem", "Initializing...");
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
        telemetry.addData("max voltage", sensor.getMaxVoltage());
        telemetry.addData("voltage", sensor.getVoltage());
        telemetry.addData("inches", sensor.getVoltage() / inchesPerVolt);
        telemetry.addData("ultrasonic", ultrasonicSensor.getDistance());

        //telemetry.addData("translated", 1023 * sensor.getVoltage()/3.3);
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}
