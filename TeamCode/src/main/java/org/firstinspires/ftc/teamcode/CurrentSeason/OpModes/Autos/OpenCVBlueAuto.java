package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines.SamplePipeline;
import org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines.ScanPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="blue cv", group="comp")
public class OpenCVBlueAuto extends AbstractAutonomous {
    public OpenCvWebcam camera;


    @Override
    public AbstractRobot instantiateRobot() {
        return null;
    }

    @Override
    public void onInit() {
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Cam"), cameraMonitorViewId);

        //set pipeline
        camera.setPipeline(new SamplePipeline());

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
            //telemetry.addData();
            while (getRuntime() < 1) {}

            camera.stopStreaming();

            ScanPipeline.State state = ScanPipeline.detectedState;

            /**
             * TODO: auto
             */

        }
    }

    @Override
    public void onStop() {

    }


}
