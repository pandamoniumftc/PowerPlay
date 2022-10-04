package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractTeleOp;
import org.firstinspires.ftc.teamcode.Robots.SetName;

@TeleOp(name="Offseason Teleop")
public class OffseasonTeleop extends AbstractTeleOp {

    @Override
    public AbstractRobot instantiateRobot() {
        return new SetName(this);
    }

    @Override
    public void onDriverUpdate() {
        
    }
}
