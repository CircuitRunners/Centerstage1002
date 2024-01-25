package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.utilities.Team;
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

    private Deque<Point>[] buffer = new Deque[ZONE_COUNT];
    private double[] areaInZone = new double[ZONE_COUNT];
    private static final int MIN_AREA = 300;
    private static final Scalar RED_LOWER_BOUND = new Scalar(0, 120, 70);
    private static final Scalar RED_UPPER_BOUND = new Scalar(10, 255, 255);
    private static final Scalar BLUE_LOWER_BOUND = new Scalar(100, 150, 0);
    private static final Scalar BLUE_UPPER_BOUND = new Scalar(140, 255, 255);
    private static final Size MORPH_KERNEL_SIZE = new Size(5, 5);

    private Mat leftZone, middleZone, rightZone;

    @Override
    public void init(Mat firstFrame) {
        for (int i = 0; i < ZONE_COUNT; i++) {
            buffer[i] = new LinkedList<>();
        }
        Arrays.fill(areaInZone, 0); // Reset the areaInZone array
    }

    int teamPropZone = 1; // The middle as default

    // Notice if you have any Mats or things needing to be released as memory,
    // Have it declared as an instance variable (and re-used), not a local variable

    @Override
    public Mat processFrame(Mat input) {
        int width = input.cols();
        int height = input.rows();

        List<Mat> matsToRelease = new ArrayList<>();

        leftZone = input.submat(new Rect(0, 0, width / 3, height));
        middleZone = input.submat(new Rect(width / 3, 0, width / 3, height));
        rightZone = input.submat(new Rect(2 * width / 3, 0, width / 3, height));

        // Add submats to the list of Mats to be released
        matsToRelease.add(leftZone);
        matsToRelease.add(middleZone);
        matsToRelease.add(rightZone);

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

        releaseMats(matsToRelease);

        return input; // wont show nothin
    }

    public int getTeamPropZone()
    {
        return teamPropZone;
    }

    public void detectObjectGeneral(Mat frame, int zoneNum, Team team) {
        try {
            Mat hsv = preProcess(frame);
            Mat mask = colorCheck(hsv, team);
            postProcess(mask, zoneNum);
            hsv.release(); // Release the hsv Mat after processing
        } catch (IllegalArgumentException e) {
            // Log to telemetry or handle the exception as appropriate
        }
    }

    private Mat preProcess(Mat frame) {
        Mat rgb = new Mat();
        Mat hsv = new Mat();

        Imgproc.cvtColor(frame, rgb, Imgproc.COLOR_RGBA2RGB);

        Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);

        List<Mat> hsvChannels = new ArrayList<>();
        Core.split(hsv, hsvChannels);

        Imgproc.equalizeHist(hsvChannels.get(2), hsvChannels.get(2));

        Core.merge(hsvChannels, hsv);

        // Release the individual channels and the RGB Mat after merging
        for (Mat channel : hsvChannels) {
            channel.release();
        }
        rgb.release(); // Don't forget to release the intermediate RGB Mat

        return hsv;
    }

    private Mat colorCheck(Mat hsv, Team team) {
        Scalar lowerBound;
        Scalar upperBound;

        if (team == Team.RED) {
            lowerBound = RED_LOWER_BOUND;
            upperBound = RED_UPPER_BOUND;
        } else if (team == Team.BLUE) {
            lowerBound = BLUE_LOWER_BOUND;
            upperBound = BLUE_UPPER_BOUND;
        } else {
            throw new IllegalArgumentException("Unsupported team color");
        }

        Mat mask = new Mat();
        Core.inRange(hsv, lowerBound, upperBound, mask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, MORPH_KERNEL_SIZE);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);

        kernel.release(); // Release the kernel after using it

        return mask;
    }

    private void postProcess(Mat mask, int zoneNum) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < MIN_AREA) continue;
            Rect rect = Imgproc.boundingRect(contour);
            if (buffer[zoneNum].size() == BUFFER_SIZE) {
                buffer[zoneNum].removeFirst();
            }
            buffer[zoneNum].addLast(new Point(rect.x, rect.y));
            areaInZone[zoneNum] += area;
        }

        hierarchy.release(); // Release the hierarchy after using it
        mask.release(); // Release the mask after using it
    }

    public void detectRedObject(Mat frame, int zoneNum) {
        detectObjectGeneral(frame, zoneNum, Team.RED);
    }

    public void detectBlueObject(Mat frame, int zoneNum) {
        detectObjectGeneral(frame, zoneNum, Team.BLUE);
    }

    private void releaseMats(List<Mat> mats) {
        for (Mat mat : mats) {
            if (mat != null) {
                mat.release();
            }
        }
        mats.clear();
    }
}