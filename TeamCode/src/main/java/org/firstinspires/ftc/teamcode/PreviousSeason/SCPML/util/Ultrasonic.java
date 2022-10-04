package org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class Ultrasonic {

    private AnalogInput sensor;
    private final double inchesPerVolt = 0.0113621;
    private final double minimumVoltage = 0.132718;
    private NanoClock clock;

    public Ultrasonic(AnalogInput ultrasonic){
        this.sensor = ultrasonic;
    }

    public double getDistance() {
        return (sensor.getVoltage() - minimumVoltage) / inchesPerVolt;
    }


}