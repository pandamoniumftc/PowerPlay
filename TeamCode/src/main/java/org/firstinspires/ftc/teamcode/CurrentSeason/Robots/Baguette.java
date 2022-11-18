package org.firstinspires.ftc.teamcode.CurrentSeason.Robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems.DeadWheelMecanum;
import org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems.LinearSlides;
import org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems.NoLocalizationMecanum;


public class Baguette extends AbstractRobot {
    public NoLocalizationMecanum drive;
    public LinearSlides intake;

    public Baguette(OpMode opMode) {
        super(opMode);

        //drive = new DeadWheelMecanum(this, "frm", "flm", "brm", "blm", "flm", "frm", "brm", 4096, 1.9685, 1);
        drive = new NoLocalizationMecanum(this, "frm", "brm", "flm", "blm");

        //AbstractRobot robot, String leftVertEC, String rightVertEC, String extEC, String clampConfig, int max, int min, double resolution, double gearRatio
        intake = new LinearSlides(this, "lvm", "rvm", "mvm", "clamp", 0, 50, 751.8, 1);

    }
}

