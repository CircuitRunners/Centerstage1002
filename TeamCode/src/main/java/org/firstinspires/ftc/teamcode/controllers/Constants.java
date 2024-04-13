package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static boolean inCompetition = true;

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
}
