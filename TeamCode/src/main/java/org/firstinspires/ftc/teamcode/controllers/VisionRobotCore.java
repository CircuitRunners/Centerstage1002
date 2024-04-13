package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.teamcode.controllers.Constants.HardwareMap.Webcam_Front;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.Team;
import org.firstinspires.ftc.teamcode.controllers.vision.TeamPropDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionRobotCore extends RobotCore{
    public VisionPortal visionPortal;               // Used to manage the video source.
    public AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    public TeamPropDetector detector;


    public VisionRobotCore(HardwareMap hardwareMap, Team team) {
        super(hardwareMap);

        detector = new TeamPropDetector(hardwareMap, true, team);
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Webcam_Front))
                .addProcessor(aprilTag)
                .build();
    }
}
