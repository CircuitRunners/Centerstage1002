package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.videoio.VideoCapture;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.Videoio;
public class TeamPropVision {
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }
    public static void main(String[] args) {
        VideoCapture cap = new VideoCapture(0);  // Use 0 to access default camera
        Mat frame = new Mat();
        while (true) {
            if (cap.read(frame)) {
                int width = frame.cols();
                Rect leftRect = new Rect(0, 0, width / 3, frame.rows());
                Rect middleRect = new Rect(width / 3, 0, width / 3, frame.rows());
                Rect rightRect = new Rect(2 * (width / 3), 0, width / 3, frame.rows());
                Mat leftZone = new Mat(frame, leftRect);
                Mat middleZone = new Mat(frame, middleRect);
                Mat rightZone = new Mat(frame, rightRect);
                detectRedObject(leftZone, 1);
                detectRedObject(middleZone, 2);
                detectRedObject(rightZone, 3);
            }
            // Stuff
        }
    }
    public static void detectRedObject(Mat frame, int zoneNum) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Core.inRange(hsv, new Scalar(0, 120, 70), new Scalar(10, 255, 255), mask1);
        Core.inRange(hsv, new Scalar(160, 120, 70), new Scalar(180, 255, 255), mask2);
        Mat mask = new Mat();
        Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, mask);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < 300) continue;
            Imgproc.drawContours(frame, Arrays.asList(contour), 0, new Scalar(0, 255, 0), 2);
            Rect rect = Imgproc.boundingRect(contour);
            Imgproc.putText(frame, "Zone " + zoneNum, new Point(rect.x, rect.y - 10), Core.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 255, 255), 2);
        }
    }
}