package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.lang.reflect.Field;
import java.util.ArrayList;

public abstract class AbstractRobot {

    public final OpMode opMode;
    public final Telemetry telemetry;
    public final HardwareMap hardwareMap;

    public ArrayList<AbstractSubsystem> subsystems;

    public Gamepad gamepad1, gamepad2;

    public BHI260IMU imu;

    public AbstractRobot(OpMode opMode) {
        this.opMode =  opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        subsystems = new ArrayList<>();

    }

    public void init() throws IllegalAccessException {
        Field[] fields = this.getClass().getDeclaredFields();

        for (Field f : fields) {
            if (AbstractSubsystem.class.isAssignableFrom(f.getType())) {
                Object obj;
                try {
                    obj = f.get(this);
                }
                catch (IllegalAccessException e) {
                    throw new IllegalAccessException("make subsystems public");
                }

                if (obj != null) {
                    subsystems.add((AbstractSubsystem)obj );
                }
            }
        }

        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        /*BHI260IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
*/
        for (AbstractSubsystem system : subsystems) {
            system.init();
        }
    }

    public final void start() {
        for (AbstractSubsystem system : subsystems) {
            system.start();
        }
    }

    public final void driverLoop() {
        for (AbstractSubsystem system : subsystems) {
            system.driverLoop();
        }
    }

    public final void stop() {
        for (AbstractSubsystem system : subsystems) {
            system.stop();
        }
    }



}
