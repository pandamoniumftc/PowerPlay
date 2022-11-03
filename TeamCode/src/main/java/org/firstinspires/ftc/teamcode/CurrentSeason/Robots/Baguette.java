package org.firstinspires.ftc.teamcode.CurrentSeason.Robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems.DeadWheelMecanum;


public class Baguette extends AbstractRobot {
    public DeadWheelMecanum drive;

    public Baguette(OpMode opMode) {
        super(opMode);

        drive = new DeadWheelMecanum(this, "frm", "flm", "brm", "blm", "flm", "frm", "brm", 4096, 1.9685, 1);
    }
}
