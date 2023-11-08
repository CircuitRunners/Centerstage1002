package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.LinkedList;
import java.util.List;

class TeamPropDetectionPipeline extends OpenCvPipeline {
    private static final int BUFFER_SIZE = 100;
    private static final int ZONE_COUNT = 3;

    private static Deque<Point>[] buffer = new Deque[ZONE_COUNT];
    private static double[] areaInZone = new double[ZONE_COUNT];
    private static final int MIN_AREA = 300;

    private Mat leftZone, rightZone, middleZone;
    private static Mat hsv, mask1, mask2, mask, kernel, hierarchy;

    @Override
    public void init(Mat firstFrame) {
        for (int i = 0; i < ZONE_COUNT; i++) {
            buffer[i] = new LinkedList<>();
        }
    }

    int teamPropZone = 1; // The middle as default

    // Notice if you have any Mats or things needing to be released as memory,
    // Have it declared as an instance variable (and re-used), not a local variable

    @Override
    public Mat processFrame(Mat input) {
        int width = input.cols();
        int height = input.rows();

        leftZone = input.submat(new Rect(0, 0, width / 3, height));
        middleZone = input.submat(new Rect(width / 3, 0, width / 3, height));
        rightZone = input.submat(new Rect(2 * width / 3, 0, width / 3, height));

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

        teamPropZone = mostLikelyZone;
        return input; // wont show nothin
    }

    public int getTeamPropZone()
    {
        return teamPropZone;
    }

    public static void detectRedObject(Mat frame, int zoneNum) {
        hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

        // Split the HSV image into separate channels
        List<Mat> hsvChannels = new ArrayList<>();
        Core.split(hsv, hsvChannels);

        // Perform histogram equalization on the V channel
        Imgproc.equalizeHist(hsvChannels.get(2), hsvChannels.get(2));

        // Merge the channels back into one image
        Core.merge(hsvChannels, hsv);

        mask1 = new Mat();
        mask2 = new Mat();
        Core.inRange(hsv, new Scalar(0, 120, 70), new Scalar(10, 255, 255), mask1);
        Core.inRange(hsv, new Scalar(160, 120, 70), new Scalar(180, 255, 255), mask2);
        mask = new Mat();
        Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, mask);

        // Perform morphological operations
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        hierarchy = new Mat();
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
    }
}
