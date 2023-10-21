package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class BeaconDetector {

    public enum BeaconTags {
        LEFT(75),
        CENTER(273),
        RIGHT(37);

        final int id;

        BeaconTags(int id) {
            this.id = id;
        }
    }

    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public BeaconDetector(HardwareMap hardwareMap, boolean useWebcamOne) {

        //Obtain the GUI element for showing the camera stream
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //Creating the camera object from the webcam
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                (useWebcamOne) ? "Webcam 1" : "Webcam 2"), cameraMonitorViewId);
        //Creating the pipeline
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        //Giving the pipeline to the camera stream
        camera.setPipeline(aprilTagDetectionPipeline);


    }

    public void startStream(){
        //Setting the opener, will happen asynchronously upon the stream starting
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void stopStream(){
        camera.stopStreaming();
        camera.closeCameraDeviceAsync(() -> {
            //do nothing
        });
    }

    public BeaconTags update() {

        BeaconTags tag = BeaconTags.CENTER;


        // Calling getDetectionsUpdate() will only return an object if there was a new frame
        // processed since the last time we called it. Otherwise, it will return null. This
        // enables us to only run logic when there has been a new frame, as opposed to the
        // getLatestDetections() method which will always return an object.
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();

        // If there's been a new frame...
        if (detections != null && detections.size() != 0) {

            if (detections.get(0).id == BeaconTags.LEFT.id) tag = BeaconTags.LEFT;
            else if (detections.get(0).id == BeaconTags.CENTER.id) tag = BeaconTags.CENTER;
            else tag = BeaconTags.RIGHT;
        }

        return tag;
    }
}