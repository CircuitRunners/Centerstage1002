package org.firstinspires.ftc.teamcode.auto.BlueAllianceAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BlueFrontAuto {
}
    private static double tile = 24;
    private static double half_tile = 12;
    private static double robot_len = 9;
    private static double square_edge = 1.5;

    private static Pose2d right_red = new Pose2d(12, -61.5, Math.toRadians(90));
    private static Pose2d left_red = new Pose2d(-36, -61.5, Math.toRadians(90));
    private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
    private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));

    public static void main(String[] args) {

        SampleMecanumDrive drive = null;
        drive.trajectorySequenceBuilder(left_blue);
        TrajectorySequence splineRightLine = drive.trajectorySequenceBuilder(left_blue)
                .splineTo(new Vector2d(-33, 35), Math.toRadians(0))
                .build();
        TrajectorySequence backAwayRightLine = drive.trajectorySequenceBuilder(left_blue)
                .back(half_tile)
                .build();
        TrajectorySequence SplineLineUpRightLine = drive.trajectorySequenceBuilder(left_blue)
                .splineTo(new Vector2d(-33, 10), Math.toRadians(180))
                .build();
        TrajectorySequence forwardToParkRightLine = drive.trajectorySequenceBuilder(left_blue)
                .forward(tile * 4 - half_tile + square_edge * 4)
                .build();
        drive.trajectorySequenceBuilder(left_blue)
        TrajectorySequence splineToCenterLine = drive.trajectorySequenceBuilder(left_blue)
                .splineTo(new Vector2d(-36, 33), Math.toRadians(-90))
                .build();
        TrajectorySequence backFromPixelCenterLine = drive.trajectorySequenceBuilder(left_blue)
                .back(6)
                .build();
        TrajectorySequence backFromPixelCenterLine = drive.trajectorySequenceBuilder(left_blue)
                .strafeRight(half_tile + square_edge * 3)
                .build();
                                            .strafeRight(half_tile + square_edge * 3)
                                            .splineTo(new Vector2d(-33, 10), Math.toRadians(90))
                                            .back(tile * 4 - half_tile + square_edge * 4)
    }