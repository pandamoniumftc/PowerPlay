package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractTeleOp;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Baguette;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.RizzBot;

@TeleOp(name="main teleop", group="comp")
public class MainTeleop extends AbstractTeleOp {

    @Override
    public AbstractRobot instantiateRobot() {
        return new RizzBot(this);
    }

    @Override
    public void onDriverUpdate() {

    }
}
