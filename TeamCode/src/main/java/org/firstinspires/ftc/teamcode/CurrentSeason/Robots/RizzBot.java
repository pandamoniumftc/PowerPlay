package org.firstinspires.ftc.teamcode.CurrentSeason.Robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems.DeadWheelMecanum;
import org.firstinspires.ftc.teamcode.CurrentSeason.SubSystems.Feeder;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Transform2D;

public class RizzBot extends AbstractRobot {
    public DeadWheelMecanum drive;
    public Feeder feeder;
    public RizzBot(OpMode opMode) {
        super(opMode);

        drive = new DeadWheelMecanum(this, "frm", "flm", "brm", "blm", "flm", "frm", "blm", 8.5, 3.78, (1.0/4096) * 3.14, new Transform2D(0, 0,0));
        feeder = new Feeder(this, "extend", "claw", "lflip1", "rflip1", "lflip2", "rflip2");
    }
}
