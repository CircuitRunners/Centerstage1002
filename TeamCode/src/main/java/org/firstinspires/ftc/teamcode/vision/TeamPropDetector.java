package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class TeamPropDetector {

    private OpenCvCamera camera;
    private TeamPropDetectionPipeline teamPropPipeline;

    public TeamPropDetector(HardwareMap hardwareMap, boolean useWebcamOne, TeamPropDetectionPipeline teamPropPipeline) {
        this.teamPropPipeline = teamPropPipeline;

        // Obtain the GUI element for showing the camera stream
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Creating the camera object from the webcam
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                (useWebcamOne) ? "Webcam 1" : "Webcam 2"), cameraMonitorViewId);

        // Giving the pipeline to the camera stream
        camera.setPipeline(this.teamPropPipeline);
    }

    public void startStream() {
        // Setting the opener, will happen asynchronously upon the stream starting
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // do nothing ~ message of the day: we love ryan
            }
        });
    }

    public void stopStream(){
        camera.stopStreaming();
        camera.closeCameraDeviceAsync(() -> {
            // do nothing ~ message of the day: we love ryan
        });
    }

    public int update() {
        int zoneDetected = teamPropPipeline.getTeamPropZone();

        return zoneDetected;
    }
}