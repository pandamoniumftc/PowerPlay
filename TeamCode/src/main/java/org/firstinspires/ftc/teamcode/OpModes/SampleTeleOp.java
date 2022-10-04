package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractTeleOp;
import org.firstinspires.ftc.teamcode.Robots.SampleRobot;

@TeleOp(name="test please work")
public class SampleTeleOp extends AbstractTeleOp {

    @Override
    public AbstractRobot instantiateRobot() {
        return new SampleRobot(this);
    }

    @Override
    public void onDriverUpdate() {
        
    }
}
