package org.firstinspires.ftc.teamcode.vision;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedBlockDetectionPipeline extends OpenCvPipeline {
    private final Scalar RED = new Scalar(0, 0, 255);
    private final Scalar lowerRed1 = new Scalar(0, 150, 100);
    private final Scalar upperRed1 = new Scalar(10, 255, 255);
    private final Scalar lowerRed2 = new Scalar(160, 150, 100);
    private final Scalar upperRed2 = new Scalar(179, 255, 255);

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(280, 540);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(960, 540);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1600, 540);
    static final int REGION_WIDTH = 75;
    static final int REGION_HEIGHT = 75;

    private Telemetry telemetry;
    private Mat HSV = new Mat();

    public RedBlockDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(Mat firstFrame) {
        Imgproc.cvtColor(firstFrame, HSV, Imgproc.COLOR_RGB2HSV);
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        BlockPosition position = detectRedBlock(HSV);
        telemetry.addData("Red Block Position", position);
        telemetry.update();
        drawRectangles(input, position);
        return input;
    }

    private BlockPosition detectRedBlock(Mat hsvImage) {
        Mat maskRed1 = new Mat();
        Mat maskRed2 = new Mat();
        Core.inRange(hsvImage, lowerRed1, upperRed1, maskRed1);
        Core.inRange(hsvImage, lowerRed2, upperRed2, maskRed2);

        Mat maskRed = new Mat();
        Core.add(maskRed1, maskRed2, maskRed);

        BlockPosition position = analyzeRegions(maskRed);
        maskRed1.release();
        maskRed2.release();
        maskRed.release();

        return position;
    }

    private BlockPosition analyzeRegions(Mat maskRed) {
        int redAmount1 = Core.countNonZero(maskRed.submat(new Rect(REGION1_TOPLEFT_ANCHOR_POINT, new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT))));
        int redAmount2 = Core.countNonZero(maskRed.submat(new Rect(REGION2_TOPLEFT_ANCHOR_POINT, new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT))));
        int redAmount3 = Core.countNonZero(maskRed.submat(new Rect(REGION3_TOPLEFT_ANCHOR_POINT, new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT))));

        if (redAmount1 > redAmount2 && redAmount1 > redAmount3) {
            return BlockPosition.LEFT;
        } else if (redAmount2 > redAmount1 && redAmount2 > redAmount3) {
            return BlockPosition.CENTER;
        } else if (redAmount3 > redAmount1 && redAmount3 > redAmount2) {
            return BlockPosition.RIGHT;
        } else {
            return BlockPosition.NONE; // No red detected or equal amounts
        }
    }

    private void drawRectangles(Mat input, BlockPosition position) {
        // Fill the detected region with a solid color
        switch (position) {
            case LEFT:
                Imgproc.rectangle(input, REGION1_TOPLEFT_ANCHOR_POINT, new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT), RED, -1);
                break;
            case CENTER:
                Imgproc.rectangle(input, REGION2_TOPLEFT_ANCHOR_POINT, new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT), RED, -1);
                break;
            case RIGHT:
                Imgproc.rectangle(input, REGION3_TOPLEFT_ANCHOR_POINT, new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT), RED, -1);
                break;
            default:
                // No fill if no block detected
                break;
        }
        // Outline rectangles for all regions for clarity
        Imgproc.rectangle(input, REGION1_TOPLEFT_ANCHOR_POINT, new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT), RED, 5);
        Imgproc.rectangle(input, REGION2_TOPLEFT_ANCHOR_POINT, new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT), RED, 5);
        Imgproc.rectangle(input, REGION3_TOPLEFT_ANCHOR_POINT, new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT), RED, 5);
    }

    public enum BlockPosition {
        LEFT, CENTER, RIGHT, NONE
    }
}
