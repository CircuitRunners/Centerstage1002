package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    private static double tile = 24;
    private static double half_tile = 12;
    private static double robot_len = 9;
    private static double square_edge = 1.5;

    private static Pose2d right= new Pose2d(12, -61.5, Math.toRadians(90));
    private static Pose2d left= new Pose2d(-36, -61.5, Math.toRadians(90));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(left)
                                .forward(square_edge + 2 * tile)
                                .strafeRight(tile * 4 - square_edge)
                                .build()
                );


//        .followTrajectorySequence(drive ->
//                drive.trajectorySequenceBuilder(left)
//                        .forward(square_edge + 2 * tile)
//                        .strafeRight(tile * 4 - square_edge)
//                        .build()
//        ); park from LEFT forward then to park one

//        .followTrajectorySequence(drive ->
//                drive.trajectorySequenceBuilder(right)
//                        .strafeRight(tile * 2 - square_edge)
//                        .build()
//        ); park from right to park zone

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}