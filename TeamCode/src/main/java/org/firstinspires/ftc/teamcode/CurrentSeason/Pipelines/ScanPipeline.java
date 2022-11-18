package org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ScanPipeline extends OpenCvPipeline {

    boolean viewportPaused;

    public enum State {
        PURPLE,
        GREEN,
        YELLOW
    }

    public static State detectedState = State.GREEN;

    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    @Override
    public Mat processFrame(Mat img)
    {
        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HLS);
        //Imgproc.cvtColor(img, img, Imgproc.COLOR_HLS2BGR);
        Mat pBinary = new Mat();
        Mat gBinary = new Mat();
        Mat yBinary = new Mat();

        /*Core.inRange(img, new Scalar(120, 20, 40), new Scalar(170, 120, 110), pBinary);
        Core.inRange(img, new Scalar(60, 15, 35), new Scalar(100, 255, 185), gBinary);
        Core.inRange(img, new Scalar(20, 10, 60), new Scalar(40, 175, 180), yBinary);*/

        Core.inRange(img, new Scalar(130, 20, 30), new Scalar(170, 100, 120), pBinary);
        Core.inRange(img, new Scalar(50, 20, 30), new Scalar(90, 115, 185), gBinary);
        Core.inRange(img, new Scalar(14, 40, 70), new Scalar(40, 160, 190), yBinary);

        /*Core.inRange(img, new Scalar(130, 0, 70), new Scalar(190, 30, 130), pBinary);
        Core.inRange(img, new Scalar(40, 70, 20), new Scalar(40, 130, 80), gBinary);
        Core.inRange(img, new Scalar(0, 220, 220), new Scalar(30, 255, 255), yBinary);*/

        Imgproc.resize(pBinary, pBinary, new Size(20, (int) Math.round((20/pBinary.size().width)*pBinary.size().height)));
        Imgproc.resize(gBinary, gBinary, new Size(20, (int) Math.round((20/gBinary.size().width)*gBinary.size().height)));
        Imgproc.resize(yBinary, yBinary, new Size(20, (int) Math.round((20/yBinary.size().width)*yBinary.size().height)));


        Mat erode = Mat.ones(3,3, CvType.CV_32F);

        Imgproc.morphologyEx(pBinary, pBinary, Imgproc.MORPH_CLOSE, erode);
        Imgproc.morphologyEx(gBinary, gBinary, Imgproc.MORPH_CLOSE, erode);
        Imgproc.morphologyEx(yBinary, yBinary, Imgproc.MORPH_CLOSE, erode);

        double p = 0;
        double g = 0;
        double y = 0;

        for (int i = 0; i < pBinary.width(); i++) {
            for (int j = 0; j < pBinary.cols(); j++) {
                if (pBinary.get(i, j) != null) {
                    p += pBinary.get(i, j)[0];
                    g += gBinary.get(i, j)[0];
                    y += yBinary.get(i, j)[0];
                }
            }
        }

        if (p >= g && p >= y) detectedState = State.PURPLE;
        if (g >= p && g >= y) detectedState = State.GREEN;
        if (y >= p && y >= g) detectedState = State.YELLOW;

        Mat intermediateMask = new Mat();

        Core.bitwise_or(pBinary, gBinary, intermediateMask);
        Core.bitwise_or(intermediateMask, yBinary, intermediateMask);

        Imgproc.cvtColor(intermediateMask, intermediateMask, Imgproc.COLOR_GRAY2BGR);

        //Imgproc.resize(img, img, new Size(20, (int) Math.round((20/intermediateMask.size().width)*intermediateMask.size().height)));
        Core.bitwise_and(intermediateMask, img, img);


        Imgproc.morphologyEx(img, img, Imgproc.MORPH_ERODE, erode);
        Imgproc.morphologyEx(img, img, Imgproc.MORPH_OPEN, erode);

        Imgproc.cvtColor(img, img, Imgproc.COLOR_HLS2BGR);

        return img;
    }

    @Override
    public void onViewportTapped()
    {
        /*
         * The viewport (if one was specified in the constructor) can also be dynamically "paused"
         * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
         * when you need your vision pipeline running, but do not require a live preview on the
         * robot controller screen. For instance, this could be useful if you wish to see the live
         * camera preview as you are initializing your robot, but you no longer require the live
         * preview after you have finished your initialization process; pausing the viewport does
         * not stop running your pipeline.
         *
         * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
         */

        /*viewportPaused = !viewportPaused;

        if(viewportPaused)
        {
            webcam.pauseViewport();
        }
        else
        {
            webcam.resumeViewport();
        }*/
    }


}
