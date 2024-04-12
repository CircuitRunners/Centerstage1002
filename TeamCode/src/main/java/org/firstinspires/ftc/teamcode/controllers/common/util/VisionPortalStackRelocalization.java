package org.firstinspires.ftc.teamcode.controllers.common.util;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.controllers.common.util.opencv.StackRelocalization;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

import java.util.ArrayList;

public class VisionPortalStackRelocalization implements VisionProcessor {

    public StackRelocalization stackRelocalization;

    public VisionPortalStackRelocalization() {
        stackRelocalization = new StackRelocalization();
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return stackRelocalization.actualProcessFrame(frame);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    public double getXError() {
        return stackRelocalization.getXError();
    }

    public double getXErrorInches() {
        return stackRelocalization.getXErrorInches();
    }

    public ArrayList<Rect> getBoundingBoxes() {
        return stackRelocalization.boundingBoxes;
    }

    public double getInputSize() {
        return stackRelocalization.output.size().area();
    }

    public String getCenterHue() {
        if (stackRelocalization.output != null) return stackRelocalization.getCenterHue();
        return "-1";
    }
}
