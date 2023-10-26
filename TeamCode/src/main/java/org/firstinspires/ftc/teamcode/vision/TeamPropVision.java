package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.*;

public class TeamPropVision {

    private Deque<Point>[] buffer = new Deque[ZONE_COUNT];
    private double[] areaInZone = new double[ZONE_COUNT];


    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    private static final int BUFFER_SIZE = 100;
    private static final int ZONE_COUNT = 3;
    private static final int MIN_AREA = 300;
    private Telemetry telemetry;

    public TeamPropVision(Telemetry telemetry) {
        this.telemetry = telemetry;
        for (int i = 0; i < ZONE_COUNT; i++) {
            buffer[i] = new LinkedList<>();
        }
    }

    public void detectRedObject(Mat frame, int zoneNum) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

        // Split the HSV image into separate channels
        List<Mat> hsvChannels = new ArrayList<>();
        Core.split(hsv, hsvChannels);

        // Perform histogram equalization on the V channel
        Imgproc.equalizeHist(hsvChannels.get(2), hsvChannels.get(2));

        // Merge the channels back into one image
        Core.merge(hsvChannels, hsv);

        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Core.inRange(hsv, new Scalar(0, 120, 70), new Scalar(10, 255, 255), mask1);
        Core.inRange(hsv, new Scalar(160, 120, 70), new Scalar(180, 255, 255), mask2);
        Mat mask = new Mat();
        Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, mask);

        // Perform morphological operations
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
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

    public int processFrame(Mat frame) {
        int width = frame.cols();
        int height = frame.rows();

        Mat leftZone = frame.submat(new Rect(0, 0, width / 3, height));
        Mat middleZone = frame.submat(new Rect(width / 3, 0, width / 3, height));
        Mat rightZone = frame.submat(new Rect(2 * width / 3, 0, width / 3, height));

        detectRedObject(leftZone, 0);
        detectRedObject(middleZone, 1);
        detectRedObject(rightZone, 2);

        int mostLikelyZone = 0;
        for (int i = 1; i < ZONE_COUNT; i++) {
            if (areaInZone[i] > areaInZone[mostLikelyZone]) {
                mostLikelyZone = i;
            }
        }

        telemetry.addData("Most likely zone", mostLikelyZone);
        telemetry.update();

        // Reset the area
        Arrays.fill(areaInZone, 0);

        return mostLikelyZone;
    }
}