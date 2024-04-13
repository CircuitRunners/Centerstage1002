package org.firstinspires.ftc.teamcode.controllers.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlueBlockDetectionPipeline extends OpenCvPipeline {
    private final Scalar BLUE = new Scalar(255, 0, 0);
    private final Scalar lowerBlue = new Scalar(100, 150, 100);
    private final Scalar upperBlue = new Scalar(140, 255, 255);

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(280, 540);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(960, 540);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1600, 540);
    static final int REGION_WIDTH = 75;
    static final int REGION_HEIGHT = 75;

    private Telemetry telemetry;
    private Mat HSV = new Mat();

    public BlueBlockDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(Mat firstFrame) {
        Imgproc.cvtColor(firstFrame, HSV, Imgproc.COLOR_RGB2HSV);
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        BlockPosition position = detectBlueBlock(HSV);
        telemetry.addData("Blue Block Position", position);
        telemetry.update();
        drawRectangles(input, position);
        return input;
    }

    private BlockPosition detectBlueBlock(Mat hsvImage) {
        Mat maskBlue = new Mat();
        Core.inRange(hsvImage, lowerBlue, upperBlue, maskBlue);

        BlockPosition position = analyzeRegions(maskBlue);
        maskBlue.release();

        return position;
    }

    private BlockPosition analyzeRegions(Mat maskBlue) {
        int blueAmount1 = Core.countNonZero(maskBlue.submat(new Rect(REGION1_TOPLEFT_ANCHOR_POINT, new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT))));
        int blueAmount2 = Core.countNonZero(maskBlue.submat(new Rect(REGION2_TOPLEFT_ANCHOR_POINT, new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT))));
        int blueAmount3 = Core.countNonZero(maskBlue.submat(new Rect(REGION3_TOPLEFT_ANCHOR_POINT, new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT))));

        if (blueAmount1 > blueAmount2 && blueAmount1 > blueAmount3) {
            return BlockPosition.LEFT;
        } else if (blueAmount2 > blueAmount1 && blueAmount2 > blueAmount3) {
            return BlockPosition.CENTER;
        } else if (blueAmount3 > blueAmount1 && blueAmount3 > blueAmount2) {
            return BlockPosition.RIGHT;
        } else {
            return BlockPosition.NONE; // No blue detected or equal amounts
        }
    }

    private void drawRectangles(Mat input, BlockPosition position) {
        // Draw and fill rectangles based on detected position
        switch (position) {
            case LEFT:
                Imgproc.rectangle(input, REGION1_TOPLEFT_ANCHOR_POINT, new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT), BLUE, -1);
                break;
            case CENTER:
                Imgproc.rectangle(input, REGION2_TOPLEFT_ANCHOR_POINT, new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT), BLUE, -1);
                break;
            case RIGHT:
                Imgproc.rectangle(input, REGION3_TOPLEFT_ANCHOR_POINT, new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT), BLUE, -1);
                break;
            default:
                // No fill if no block detected
                break;
        }
        // Outline rectangles for all regions for clarity
        Imgproc.rectangle(input, REGION1_TOPLEFT_ANCHOR_POINT, new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT), BLUE, 5);
        Imgproc.rectangle(input, REGION2_TOPLEFT_ANCHOR_POINT, new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT), BLUE, 5);
        Imgproc.rectangle(input, REGION3_TOPLEFT_ANCHOR_POINT, new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT), BLUE, 5);
    }

    public enum BlockPosition {
        LEFT, CENTER, RIGHT, NONE
    }
}
