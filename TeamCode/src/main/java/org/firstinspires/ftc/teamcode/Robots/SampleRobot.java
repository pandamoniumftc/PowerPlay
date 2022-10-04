package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeAndSpinner;
import org.firstinspires.ftc.teamcode.SubSystems.TurnMotorON;
import org.firstinspires.ftc.teamcode.SubSystems.TurnServoOn;
import org.firstinspires.ftc.teamcode.SubSystems.UltrasonicSubSystem;

public class SampleRobot extends AbstractRobot {

    //public TurnMotorON motorON;
    //public TurnServoOn servoON;
    //public UltrasonicSubSystem ultrasonic;
    //public ColorSensorSubsystem color;
    public IntakeAndSpinner intake;
    public SampleRobot(OpMode opMode) {
        super(opMode);
        //servoON = new TurnServoOn(this, "testservo");
        //ultrasonic = new UltrasonicSubSystem(this, "ultrasonic");
        //motorON = new TurnMotorON(this, "testmotor");
        //color = new ColorSensorSubsystem(this, "color_sensor");
        //intake = new IntakeAndSpinner(this, "intake_motor");
    }
}
