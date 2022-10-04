package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.SCPML.util.Encoder;
import org.firstinspires.ftc.teamcode.SCPML.util.PID;
import org.firstinspires.ftc.teamcode.SubSystems.ColorSensorOuttakeArmIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.DuckSpinnerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeAndSpinner;
import org.firstinspires.ftc.teamcode.SubSystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.SCPMLSubsystemTest;

public class SetName extends AbstractRobot {

    //public ManualOuttakeArm outtakeArm;
    public IntakeAndSpinner intake;
    //outtakeArm = new ManualOuttakeArm(this, "rYaw", "lYaw", "rArm", "lArm", "upper_arm", "latch");

    //cv = new OpenCVSubsystem(this);
//autoOuttakeArm = new AutomaticOuttakeArm(this, "rYaw", "lYaw", "rArm", "lArm", "upper_arm", "latch");
    public MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this, "frm", "brm", "flm", "blm");
    //public AutomaticOuttakeArm autoOuttakeArm;
    public ColorSensorOuttakeArmIntake arm;
    //public OpenCVSubsystem cv;
    public SCPMLSubsystemTest scpmlSubsystemTest;
    //public ColorSensorSubsystem color;\
    //public DuckSpinnerSubsystem duck;

    public SetName(OpMode opMode) {
        super(opMode);

        //ToDo: tune with new robot
        //PID
        PID.setConstantsTurn( 1, 1, 1);
        PID.setConstantsMove( 1, 1, 1);
        PID.setConstantsArm( 0.025, 0, 0.015);

        intake = new IntakeAndSpinner(this, "intake_motor");

        //color = new ColorSensorSubsystem(this, "color_sensor");

        arm = new ColorSensorOuttakeArmIntake(this, "rYaw", "lYaw", "rArm", "lArm", "upper_arm", "latch","color_sensor","intake_motor", "turret_encoder" ,90);
        scpmlSubsystemTest = new SCPMLSubsystemTest(this, (LinearOpMode)opMode);

        //duck = new DuckSpinnerSubsystem(this, "intake_motor");


    }
}
