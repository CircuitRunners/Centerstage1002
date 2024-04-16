package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.controllers.common.utilities.TriPose;

@Config
public class Constants {
    public static boolean inCompetition = false;

    // Front means the deposit side of the robot
    public static class HardwareMap {
        public static String
            Webcam_Front = "Webcam 1",
            Webcam_Back = "",
            iw = "yo";

    }

    public static class IntakeCommandConstants {
        public static double
            DETECTION_THRESHOLD = 4.0, // Threshold for the distance sensor
            REQUIRED_TIME_MS = 200,
            FINISH_LOWSPEED_THRESHOLD = 200,
            OUTTAKE_TIME_ROBOT = 1350;  // consider reducing if need faster cycle times
    }

    @Config
    public static class AutoPoses {
        @Config
        public static class RED_STAGE {
            public static Pose2d r_s_startPos = new Pose2d(15.75, -63, Math.toRadians(90.00));
            public static TriPose pixelPositions = new TriPose(
                    new Pose2d(0,0, Math.toRadians(0)), // Left
                    new Pose2d(15.137,-35.148, Math.toRadians(111.035)), // Middle 15.137	-35.148	111.035
                    new Pose2d(0,0, Math.toRadians(0))  // Right
            );
            public static TriPose backPoint = new TriPose(
                    new Pose2d(0,0, Math.toRadians(0)), // Left
                    new Pose2d(15.407,-40.374, Math.toRadians(90)), // Middle 15.407	-40.374	90
                    new Pose2d(0,0, Math.toRadians(0))  // Right
            );
            public static TriPose boardPositions = new TriPose(
                    new Pose2d(50.253,-33.104, Math.toRadians(0)), // Left
                    new Pose2d(50.253,-38.804, Math.toRadians(0)), // Middle EMPIRICALLY
                    new Pose2d(50.253,-43.804, Math.toRadians(0))  // Right
            );
            public static Pose2d stagePosition = new Pose2d(50.33, -14, Math.toRadians(-90));
            public static Pose2d parkPosition = new Pose2d(62.75, -14, Math.toRadians(-90));

        }
        @Config
        public static class RED_AUDIENCE {
            public static Pose2d r_a_startPos = new Pose2d(-39.3, -63, Math.toRadians(90.00));
            public static TriPose pixelPositions = new TriPose(
                    new Pose2d(-45.645,-34.758, Math.toRadians(90)), // Left // y is guessed
                    new Pose2d(-35.904,-34.936, Math.toRadians(90)), // Middle
                    new Pose2d(-29.456,-35.005, Math.toRadians(60))  // Right
            );
            public static Pose2d stackPositions = new Pose2d(-56.197,-35.176, Math.toRadians(0));
            public static Pose2d beforeGoingThroughBridge = new Pose2d(-38.318,-59.914, Math.toRadians(0));
            public static Pose2d afterGoingThroughBridge = new Pose2d(32.758,-59.914, Math.toRadians(0));
            public static TriPose preBoard = new TriPose(
                    new Pose2d(45,-34, Math.toRadians(0)), // Left
                    new Pose2d(44.875,-40.960, Math.toRadians(0)), // Middle
                    new Pose2d(44.875,-46.00, Math.toRadians(0))  // Right
            );
            public static TriPose toBoard = new TriPose(
                    new Pose2d(49,-34, Math.toRadians(0)), // Left
                    new Pose2d(48.875,-40.960, Math.toRadians(0)), // Middle
                    new Pose2d(48.875,-46.00, Math.toRadians(0))  // Right
            );
            // iteration 1
            public static Pose2d toPark = new Pose2d(45,-61, Math.toRadians(270));
            // iteration 2
//            public static Pose2d toPark = new Pose2d(45,-61, Math.toRadians(270));
        }
    }
}


