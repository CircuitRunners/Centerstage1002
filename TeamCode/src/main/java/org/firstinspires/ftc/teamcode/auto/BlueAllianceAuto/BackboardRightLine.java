package org.firstinspires.ftc.teamcode.auto.BlueAllianceAuto;


import com.acmerobotics.roadrunner.geometry.Pose2d;

public class BlueRightLinePixelPlacement {
    private static double tile = 24;
    private static double half_tile = 12;
    private static double robot_len = 9;
    private static double square_edge = 1.5;

    private static Pose2d right_red = new Pose2d(12, -61.5, Math.toRadians(90));
    private static Pose2d left_red = new Pose2d(-36, -61.5, Math.toRadians(90));
    private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
    private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));

        public static void main(String[] args) {
            
                                    drive.trajectorySequenceBuilder(right_blue)
                                            .splineTo(new Vector2d(31, 30), Math.toRadians(180))
                                            .back(5)
                                            .strafeRight(tile + square_edge * 3)
                                            .back(tile)
                                            .build()
                            );