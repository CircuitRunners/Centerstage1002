package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectorySequences {

    private static Pose2d right_red = new Pose2d(12, -61.5, Math.toRadians(90));
    private static Pose2d left_red = new Pose2d(-36, -61.5, Math.toRadians(90));
    private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
    private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));



    TrajectorySequence rightPark = drive.trajectorySequenceBuilder(right)
            .strafeLeft(powerFullMultiplier*(tile * 2 - square_edge))
            .build();

    // CHANGE THIS RIGHT VALUE THIS IS BAD
    TrajectorySequence backOff = drive.trajectorySequenceBuilder(right)
            .back(half_tile)
            .build();

    TrajectorySequence leftPark = drive.trajectorySequenceBuilder(left)
            .forward(powerFullMultiplier*(square_edge + 2 * tile))
            .strafeRight(powerFullMultiplier*(tile * 4 - square_edge))
            .build();

}
