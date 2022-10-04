package org.firstinspires.ftc.teamcode.PreviousSeason.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.PreviousSeason.opencvstuff.ObjectDetectionPipeline;
import org.firstinspires.ftc.teamcode.PreviousSeason.Robots.SetName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="CV")
public class OpenCVAuto extends AbstractAutonomous {
    OpenCvWebcam camera;

    @Override
    public void onInit() {
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Cam"), cameraMonitorViewId);

//set pipeline
        camera.setPipeline(new ObjectDetectionPipeline());

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
        SetName robot = (SetName) getRobot();


    }

    @Override
    public AbstractRobot instantiateRobot() {
        return new SetName(this);

    }


}
