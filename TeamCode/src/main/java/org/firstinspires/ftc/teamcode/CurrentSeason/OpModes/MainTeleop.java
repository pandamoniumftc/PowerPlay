package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractTeleOp;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Baguette;

@TeleOp(name="main teleop", group="comp")
public class MainTeleop extends AbstractTeleOp {

    @Override
    public AbstractRobot instantiateRobot() {
        return new Baguette(this);
    }
}
