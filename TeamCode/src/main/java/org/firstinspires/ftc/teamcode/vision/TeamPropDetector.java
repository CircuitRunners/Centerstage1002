package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utilities.PropLocation;
import org.firstinspires.ftc.teamcode.utilities.Team;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class TeamPropDetector {

    private OpenCvCamera camera;
    private TeamPropDetectionPipeline teamPropPipeline;

    public TeamPropDetector(HardwareMap hardwareMap, boolean useWebcamOne, Team team) {
        this.teamPropPipeline = new TeamPropDetectionPipeline();
        switch (team) {
            case RED: {
                teamPropPipeline.setTeam(Team.RED);
                break;
            }
            case BLUE: {
                teamPropPipeline.setTeam(Team.BLUE);
                break;
            }
        }

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
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
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

    public PropLocation update() {
        PropLocation zoneDetected = teamPropPipeline.getTeamPropZone();

        return zoneDetected;
    }
}