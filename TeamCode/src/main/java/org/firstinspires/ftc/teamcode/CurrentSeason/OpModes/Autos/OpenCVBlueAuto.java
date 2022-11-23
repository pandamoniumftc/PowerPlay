package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines.SamplePipeline;
import org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines.ScanPipeline;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Baguette;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="right cv", group="comp")
public class OpenCVBlueAuto extends AbstractAutonomous {
    public OpenCvWebcam camera;
    public Baguette robot;

    @Override
    public AbstractRobot instantiateRobot() {
        robot = new Baguette(this);
        return robot;
    }

    @Override
    public void onInit() {
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Cam"), cameraMonitorViewId);

        //set pipeline
        camera.setPipeline(new ScanPipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    @Override
    public void autonomous() {

        waitForStart();

        resetRuntime();
        if (opModeIsActive()) {
            ScanPipeline.State state = ScanPipeline.detectedState;
            if (state == ScanPipeline.State.PURPLE) {
                robot.drive.setMotorPower(0, -1, 0, 0.3);
                sleep (1500);
                robot.drive.setMotorPower(0,0,0,0);
                sleep(500);
                robot.drive.setMotorPower(1, 0, 0, 0.5);
                sleep(1500);
                robot.drive.setMotorPower(0,0,0,0);
            }
            if (state == ScanPipeline.State.GREEN) {
                robot.drive.setMotorPower(0, -1, 0, 0.3);
                sleep (1500);
                robot.drive.setMotorPower(0,0,0,0);
                sleep(500);
            }
            if (state == ScanPipeline.State.YELLOW) {
                robot.drive.setMotorPower(0, -1, 0, 0.3);
                sleep (1500);
                robot.drive.setMotorPower(0,0,0,0);
                sleep(500);
                robot.drive.setMotorPower(-1, 0, 0, 0.5);
                sleep(1500);
                robot.drive.setMotorPower(0,0,0,0);
            }

            camera.stopStreaming();
            camera.closeCameraDevice();
        }


    }

    @Override
    public void onStop() {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }


}
