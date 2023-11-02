package org.firstinspires.ftc.teamcode.auto.BlueAllianceAuto;


import static com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.back;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BlueCenterLinePixelPlacement {
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
                                    drive.trajectorySequenceBuilder(right_blue)
                                            .splineTo(new Vector2d(12, 33), Math.toRadians(-90))
                                            .back(tile + square_edge)
                                            .strafeLeft(tile * 2)
                                            .build()
                            );