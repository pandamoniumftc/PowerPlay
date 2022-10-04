package org.firstinspires.ftc.teamcode.PreviousSeason.opencvstuff;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;


public class ObjectDetectionPipeline extends OpenCvPipeline
{
    public static Mat src = new Mat();
    public static Mat src_mask = new Mat();
    public static Mat dst = new Mat();
    boolean viewportPaused;

    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    @Override
    public Mat processFrame(Mat img) {
        Mat src_converted = new Mat();
        Mat src_blurred = new Mat();
        Point centroid = new Point();
        final Point img_center = new Point(Math.round(img.width()), Math.round(img.height() / 2));
        src = img;
        dst = src.clone();
        // src = filtering on it
        // dst = clone of src, adding bounding box, centroid, and stats on it

        //Imgproc.GaussianBlur(src, src_blurred, new Size(7, 7), 0, 0, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C);
        // blurring img
        Imgproc.cvtColor(src, src_converted, Imgproc.COLOR_BGR2HSV);
        Core.inRange(src_converted, new Scalar(0, 120, 150), new Scalar(120, 255, 255), src_mask);
        // converting img into hsv color space in order to detect cube and sphere

        Mat kernel = Mat.ones(new Size(5, 5), 1);
        Imgproc.dilate(src_mask, src_mask, kernel);
        Imgproc.erode(src_mask, src_mask, kernel);
        // dilated and erode the image to add more definition to the edges

        Mat hierarchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        MatOfPoint2f approx = new MatOfPoint2f();
        int[] hierarchy_array = new int[4];
        // created an MatOfPoint array for retrieving and storing all possible contours and a Mat named hierarchy to store information about contour hierarchy
        // created an MatOfPoint2f array for storing the approximated curve output of the original curve in the MatOfPoint array contours

        Imgproc.findContours(src_mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        // find contours in mask

        HashMap<Integer, Point> distance_data = new HashMap<>();
        List<Integer> distance_values = new ArrayList<>();
        // created a hashmap and lists for storing together distance and specific centroid/points together to gain easy access

        for (int i = 0; i < contours.size(); i++) {
            // this for loop is taking every contour in the contours array based on a counter of i
            // this keep adding on to the counter of i (hence the i++) until all contour are read (hence the i < contours.size())

            double peri = Imgproc.arcLength(new MatOfPoint2f(contours.get(i).toArray()), true);
            // stores the arc length or the perimeter of the contours
            Rect boundingRect = Imgproc.boundingRect(contours.get(i));
            // stores the bounding rectangle of all contours
            double contour_Area = Imgproc.contourArea(contours.get(i));
            // stores the area of the contour
            double aspectRatio = (double) boundingRect.width / boundingRect.height;
            // stores the aspect ratio of the bounding box of each contour
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), approx, 0.01 * peri, true);
            // takes an approximation of the curve based on the Douglas-Peucker algorithm, resulting in a simplified version of the original curve with fewer points
            int sides = approx.toList().size();
            // stores the number of sides of each approximated curve

            for (int h = 0; h < hierarchy.rows(); h++) {
                hierarchy.get(h, 0, hierarchy_array);
            }

            int next = hierarchy_array[0];
            int previous = hierarchy_array[1];
            int child = hierarchy_array[2];
            int parent = hierarchy_array[3];

            // gets and stores specific information about the contour hierarchy (next and previous contours, its child or parent contours)

            if ((contour_Area > 500 && contour_Area < 50000) && (aspectRatio > 0.5 && aspectRatio < 1.1) && (previous == -1 && parent == -1)) {
                // conditionals for the contour, checking to see if the contour area fits the required range, if the contour's bounding box fits the required aspect ratio range, AND if the contour has no previous contour or parent
                // used to filter out any unneeded noise or patches

                if ((sides >= 5 && sides <= 15) && (aspectRatio >= 0.6 && aspectRatio <= 1.1 || aspectRatio == 1)) {
                    // if the approximated curve has 4-15 sides AND has an aspect ratio between 0.6 and 1.1 OR is exactly 1
                    // it will assume that an object (cube or sphere) has been detected

                    final Moments moments = Imgproc.moments(approx);
                    centroid.x = Math.round(moments.get_m10() / moments.get_m00());
                    centroid.y = Math.round(moments.get_m01() / moments.get_m00());
                    // finds the coordinates of the centroid of each contour

                    Imgproc.rectangle(dst, boundingRect, new Scalar(0, 0, 255), 3);
                    Imgproc.circle(dst, centroid, 1, new Scalar(0, 0, 0), 3);
                    // finally draws the bounding box and centroid of the contour
                    // centroid is used to align the robot to the cube based on the x coordinate of the robot's camera and the cube

                    Imgproc.circle(dst, img_center, 1, new Scalar(0, 0, 0), 3);
                    // draws the center of the camera

                    double a = Math.pow(centroid.x - img_center.x, 2);
                    double b = Math.pow(centroid.y - img_center.y, 2);
                    int distance = (int) Math.round((Math.sqrt(a + b)));
                    // finds the distance of between one centroid and the center point for each centroid drawn

                    distance_data.put(distance, new Point(centroid.x, centroid.y));
                    Imgproc.line(dst, centroid, img_center, new Scalar(0, 0, 255), 1);
                    distance_values.add(distance);
                    // finally puts the data into the hashmap
                }
            }
        }
        return dst;
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
