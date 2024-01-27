package org.firstinspires.ftc.teamcode.vision;

//import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.BOTTOM_CUTOFF_PIXELS_PROPORTION;
//import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.BUFFER_SIZE;
//import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.MIN_AREA;
//import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.TOP_CUTOFF_PIXELS_PROPORTION;
//import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.ZONE_COUNT;

import org.firstinspires.ftc.teamcode.utilities.PropLocation;
import org.firstinspires.ftc.teamcode.utilities.Team;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.nio.channels.Pipe;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.LinkedList;
import java.util.List;

class TeamPropDetectionPipelineRed extends OpenCvPipeline {
    private static final int BUFFER_SIZE = 100;
    private static final int ZONE_COUNT = 3;
    private static final double TOP_CUTOFF_PIXELS_PROPORTION = 0.3;
    private static final double BOTTOM_CUTOFF_PIXELS_PROPORTION = 0.05;
    private static final int MIN_AREA = 300;


    public static Deque<Point>[] buffer = new Deque[ZONE_COUNT];
    public static double[] areaInZone = new double[ZONE_COUNT];


    private Mat leftZone, middleZone, rightZone;

    @Override
    public void init(Mat firstFrame) {
        for (int i = 0; i < ZONE_COUNT; i++) {
            buffer[i] = new LinkedList<>();
        }
    }

    PropLocation teamPropZone = PropLocation.RIGHT; // The middle as default

    // Notice if you have any Mats or things needing to be released as memory,
    // Have it declared as an instance variable (and re-used), not a local variable

    @Override
    public Mat processFrame(Mat input) {
        int width = input.cols();
        int height = input.rows();

        List<Mat> matsToRelease = new ArrayList<>();

        int topPixelsToRemove = (int) (height * TOP_CUTOFF_PIXELS_PROPORTION);
        int bottomPixelsToRemove = (int) (height * BOTTOM_CUTOFF_PIXELS_PROPORTION);

        // Crop the image to remove the top and bottom pixels.
        input = new Mat(input, new Range(topPixelsToRemove, height - bottomPixelsToRemove), new Range(0, width));

        leftZone = input.submat(new Rect(0, 0, width / 3, height));
        middleZone = input.submat(new Rect(width / 3, 0, width / 3, height));
        rightZone = input.submat(new Rect(2 * width / 3, 0, width / 3, height));

        // Add submats to the list of Mats to be released
        matsToRelease.add(leftZone);
        matsToRelease.add(middleZone);
        matsToRelease.add(rightZone);
//        matsToRelease.add(croppedInput);

        detectRedObject(leftZone, 0);
        detectRedObject(middleZone, 1);
        detectRedObject(rightZone, 2);

        int mostLikelyZone = 0;
        for (int i = 1; i < ZONE_COUNT; i++) {
            if (areaInZone[i] > areaInZone[mostLikelyZone]) {
                mostLikelyZone = i;
            }
        }

        // Reset the area
        Arrays.fill(areaInZone, 0);

        switch (mostLikelyZone) {
            case 0: {
                teamPropZone = PropLocation.LEFT;
                break;
            }
            case 1: {
                teamPropZone = PropLocation.MIDDLE;
                break;
            }
            case 2: {
                teamPropZone = PropLocation.RIGHT;
                break;
            }
        }

        releaseMats(matsToRelease);

        return input;
    }

    public PropLocation getTeamPropZone()
    {
        return teamPropZone;
    }

    public void releaseMats(List<Mat> mats) {
        for (Mat mat : mats) {
            if (mat != null) {
                mat.release();
            }
        }
    }

    public void detectRedObject(Mat frame, int zoneNum) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_RGB2HSV);

        // Split the HSV image into separate channels
        List<Mat> hsvChannels = new ArrayList<>();
        Core.split(hsv, hsvChannels);

        // Perform histogram equalization on the V channel
        Imgproc.equalizeHist(hsvChannels.get(2), hsvChannels.get(2));

        // Merge the channels back into one image
        Core.merge(hsvChannels, hsv);

        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat mask = new Mat();
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Mat hierarchy = new Mat();
        // All mats to release at the end
        List<Mat> matsToRelease = Arrays.asList(hsv, mask1, mask2, mask, kernel, hierarchy);

        Core.inRange(hsv, new Scalar(0, 120, 70), new Scalar(10, 255, 255), mask1);
        Core.inRange(hsv, new Scalar(160, 120, 70), new Scalar(180, 255, 255), mask2);
        Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, mask);

        // Perform morphological operations
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < MIN_AREA) continue;
            Rect rect = Imgproc.boundingRect(contour);
            if (buffer[zoneNum].size() == BUFFER_SIZE) {
                buffer[zoneNum].removeFirst();  // Remove the oldest position if the buffer is full
            }
            buffer[zoneNum].addLast(new Point(rect.x, rect.y));  // Add the new position to the end of the buffer
            areaInZone[zoneNum] += area;
        }

        releaseMats(matsToRelease);
    }
}