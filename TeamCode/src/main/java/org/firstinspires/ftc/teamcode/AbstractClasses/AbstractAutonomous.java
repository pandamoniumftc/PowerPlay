package org.firstinspires.ftc.teamcode.AbstractClasses;

public abstract class AbstractAutonomous extends AbstractOpMode{

    public AbstractRobot robot;
    public abstract void autonomous();

    @Override
    public final void runOpMode() {

        super.runOpMode();
        robot = getRobot();

        try {

            robot.init();
            onInit();

            while(!isStarted() && !isStopRequested()) {}

            robot.start();
            autonomous();

            onStop();
            robot.stop();

        }
        catch (IllegalAccessException e) {
            e.printStackTrace();
        }
    }

}
