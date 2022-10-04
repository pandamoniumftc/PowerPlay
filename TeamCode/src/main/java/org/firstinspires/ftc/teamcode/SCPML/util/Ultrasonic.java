package org.firstinspires.ftc.teamcode.SCPML.util;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.teamcode.SCPML.util.NanoClock;

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