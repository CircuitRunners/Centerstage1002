package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    private static double tile = 24;
    private static double half_tile = 12;
    private static double robot_len = 9;
    private static double square_edge = 1.5;

    private static Pose2d right_red = new Pose2d(12, -61.5, Math.toRadians(90));
    private static Pose2d left_red = new Pose2d(-36, -61.5, Math.toRadians(90));
    private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
    private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
               //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                  .followTrajectorySequence(drive ->
                                   drive.trajectorySequenceBuilder(right_red)
                                           .forward(1.25*tile - square_edge)
                                           .back(1.1*tile)
                                           .strafeRight(tile*2)
                                           .build()
                        );
                      /* .followTrajectorySequence(drive ->
                               drive.trajectorySequenceBuilder(left_blue)
                                       .splineTo(new Vector2d(-46, 38), Math.toRadians(-90))
                                       .back(6)
                                       .strafeLeft(half_tile - square_edge)
                                       .forward(tile + half_tile)
                                       .splineTo(new Vector2d(-30, 8), Math.toRadians(0))
                                       .forward(tile * 4 - half_tile)
                                       .build()
                               );*///purple to left & park
                      /* .followTrajectorySequence(drive ->
                               drive.trajectorySequenceBuilder(left_red)
                                       .splineToLinearHeading(new Pose2d(-34, -34), Math.toRadians(90))
                                       .back(2)
                                       .strafeLeft(tile)
                                       .forward(tile* 4 - square_edge)
                                       .build()
                               ); *///purple to right & park



                /*.followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(left)
                                .forward(1.25*tile - square_edge)
                                .back(2)
                                .strafeLeft(0.5*tile)
                                .forward(tile)
                                .strafeRight(tile * 4.5 - square_edge)
                                .build()
                );*///purple to middle &park


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
        //private static Pose2d right= new Pose2d(12, -61.5, Math.toRadians(90));
        //private static Pose2d left= new Pose2d(-36, -61.5, Math.toRadians(90));

        //public static void main(String[] args) {
            //MeepMeep meepMeep = new MeepMeep(800);

            //RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    //.followTrajectorySequence(drive ->
                                    //drive.trajectorySequenceBuilder(left_red)
                                            //.splineTo(new Vector2d(-32, -35), Math.toRadians(0))
                                            //.back(5)
                                            //.strafeLeft(tile)
                                            //.forward(tile * 4 - square_edge * 2)
                                            //.build()




        //This is red team side opposite backboard, left line
        //private static Pose2d right_red = new Pose2d(12, -61.5, Math.toRadians(90));
        //private static Pose2d left_red = new Pose2d(-36, -61.5, Math.toRadians(90));
        //private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
//        private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));

        //public static void main(String[] args) {
           // MeepMeep meepMeep = new MeepMeep(800);

          //  RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            //        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
             //       .followTrajectorySequence(drive ->
              //                      drive.trajectorySequenceBuilder(left_red)
              //                              .splineTo(new Vector2d(-46, -43), Math.toRadians(90))
              //                              .back(4)
               //                             .strafeRight(half_tile - square_edge)
                //                            .forward(tile + half_tile)
                //                            .strafeRight(tile * 4 - square_edge * 2)
                 //                           .build()
 //                                    );
        //This is red team side opposite backboard, right line
        //private static Pose2d right= new Pose2d(12, -61.5, Math.toRadians(90));
        //private static Pose2d left= new Pose2d(-36, -61.5, Math.toRadians(90));
        //private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
//        private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));

        //public static void main(String[] args) {
        //MeepMeep meepMeep = new MeepMeep(800);

        //RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
        //.followTrajectorySequence(drive ->
        //drive.trajectorySequenceBuilder(left_red)
        //.splineTo(new Vector2d(-32, -35), Math.toRadians(0))
        //.back(5)
        //.strafeLeft(tile)
        //.forward(tile * 4 - square_edge * 2)
        //.build()
        //);

        //This is red team side opposite backboard, center line
        //private static Pose2d right_red = new Pose2d(12, -61.5, Math.toRadians(90));
        //private static Pose2d left_red = new Pose2d(-36, -61.5, Math.toRadians(90));
//        private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
//        private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));

        //public static void main(String[] args) {
            //MeepMeep meepMeep = new MeepMeep(800);

            //RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    //.followTrajectorySequence(drive ->
                                    //drive.trajectorySequenceBuilder(left_red)
                                            //.splineTo(new Vector2d(-42, -25), Math.toRadians(0))
                                            //.back(5)
                                            //.strafeLeft(half_tile)
                                            //.forward(tile * 4  + half_tile - square_edge * 2)
                                            //.build()
        //This is red team side backboard, right line
//        private static Pose2d right_red = new Pose2d(12, -61.5, Math.toRadians(90));
//        private static Pose2d left_red = new Pose2d(-36, -61.5, Math.toRadians(90));
//        private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
//        private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));
//
//        public static void main(String[] args) {
//            MeepMeep meepMeep = new MeepMeep(800);
//
//            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                    .followTrajectorySequence(drive ->
//                                    drive.trajectorySequenceBuilder(right)
//                                            .splineTo(new Vector2d(31, -30), Math.toRadians(180))
//                                            .back(4)
//                                            .strafeLeft(tile + square_edge * 3)
//                                            .back(tile)
//                                            .build()
        //                            );
        //This is red team side backboard, left line
//        private static double tile = 24;
//        private static double half_tile = 12;
//        private static double robot_len = 9;
//        private static double square_edge = 1.5;
//
//        private static Pose2d right_red = new Pose2d(12, -61.5, Math.toRadians(90));
//        private static Pose2d left_red = new Pose2d(-36, -61.5, Math.toRadians(90));
//        private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
//        private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));
//
//        public static void main(String[] args) {
//            MeepMeep meepMeep = new MeepMeep(800);
//
//            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                    .followTrajectorySequence(drive ->
//                                    drive.trajectorySequenceBuilder(right_red)
//                                            .splineTo(new Vector2d(9, -35), Math.toRadians(180))
//                                            .back(4)
//                                            .strafeLeft(tile)
//                                            .back(tile * 2 - square_edge* 2)
//                                            .build()
        //                            );
        //This is red team side backboard, center line
//        private static Pose2d right= new Pose2d(12, -61.5, Math.toRadians(90));
//        private static Pose2d left= new Pose2d(-36, -61.5, Math.toRadians(90));
//        private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
//        private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));
//
//        public static void main(String[] args) {
//            MeepMeep meepMeep = new MeepMeep(800);
//
//            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                    .followTrajectorySequence(drive ->
//                                    drive.trajectorySequenceBuilder(right_red)
//                                            .splineTo(new Vector2d(12, -32), Math.toRadians(90))
//                                            .back(tile + square_edge)
//                                            .strafeRight(tile * 2 - square_edge * 2)
//                                            .build()
        //                            );
        //This is blue team side backboard, right line
//        private static Pose2d right_red = new Pose2d(12, -61.5, Math.toRadians(90));
//        private static Pose2d left_red = new Pose2d(-36, -61.5, Math.toRadians(90));
//        private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
//        private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));
//
//        public static void main(String[] args) {
//            MeepMeep meepMeep = new MeepMeep(800);
//
//            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                    .followTrajectorySequence(drive ->
//                                    drive.trajectorySequenceBuilder(right_blue)
//                                            .splineTo(new Vector2d(31, 30), Math.toRadians(180))
//                                            .back(5)
//                                            .strafeRight(tile + square_edge * 3)
//                                            .back(tile)
//                                            .build()
        //                            );
        //This is blue team side backboard, left line
//        private static Pose2d right_red = new Pose2d(12, -61.5, Math.toRadians(90));
//        private static Pose2d left_red = new Pose2d(-36, -61.5, Math.toRadians(90));
//        private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
//        private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));
//
//        public static void main(String[] args) {
//            MeepMeep meepMeep = new MeepMeep(800);
//
//            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                    .followTrajectorySequence(drive ->
//                                    drive.trajectorySequenceBuilder(right_blue)
//                                            .splineTo(new Vector2d(9, 35), Math.toRadians(180))
//                                            .back(5)
//                                            .strafeRight(tile)
//                                            .back(tile + half_tile + square_edge * 3)
//                                            .build()
        //                            );
        //This is blue team side backboard, center line
//        private static Pose2d right_red = new Pose2d(12, -61.5, Math.toRadians(90));
//        private static Pose2d left_red = new Pose2d(-36, -61.5, Math.toRadians(90));
//        private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
//        private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));
//
//        public static void main(String[] args) {
//            MeepMeep meepMeep = new MeepMeep(800);
//
//            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                    .followTrajectorySequence(drive ->
//                                    drive.trajectorySequenceBuilder(right_blue)
//                                            .splineTo(new Vector2d(12, 33), Math.toRadians(-90))
//                                            .back(tile + square_edge)
//                                            .strafeLeft(tile * 2)
//                                            .build()
        //                            );
        //This is blue team side opposite backboard, right line
//        private static Pose2d right_red = new Pose2d(12, -61.5, Math.toRadians(90));
//        private static Pose2d left_red = new Pose2d(-36, -61.5, Math.toRadians(90));
//        private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
//        private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));
//
//        public static void main(String[] args) {
//            MeepMeep meepMeep = new MeepMeep(800);
//
//            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                    .followTrajectorySequence(drive ->
//                                    drive.trajectorySequenceBuilder(left_blue)
//                                            .splineTo(new Vector2d(-33, 35), Math.toRadians(0))
//                                            .back(half_tile)
//                                            .splineTo(new Vector2d(-33, 10), Math.toRadians(180))
//                                            .forward(tile * 4 - half_tile + square_edge * 4)
//                                            .build()
        //                            );
        //This is blue team side opposite backboard, center line
//        private static Pose2d right_red = new Pose2d(12, -61.5, Math.toRadians(90));
//        private static Pose2d left_red = new Pose2d(-36, -61.5, Math.toRadians(90));
//        private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
//        private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));
//
//        public static void main(String[] args) {
//            MeepMeep meepMeep = new MeepMeep(800);
//
//            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                    .followTrajectorySequence(drive ->
//                                    drive.trajectorySequenceBuilder(left_blue)
//                                            .splineTo(new Vector2d(-36, 33), Math.toRadians(-90))
//                                            .back(6)
//                                            .strafeRight(half_tile + square_edge * 3)
//                                            .splineTo(new Vector2d(-33, 10), Math.toRadians(90))
//                                            .back(tile * 4 - half_tile + square_edge * 4)
//                                            .build()
        //                            );
        //This is blue team side opposite backboard, center line
//        private static Pose2d right_red = new Pose2d(12, -61.5, Math.toRadians(90));
//        private static Pose2d left_red = new Pose2d(-36, -61.5, Math.toRadians(90));
//        private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
//        private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));
//
//        public static void main(String[] args) {
//            MeepMeep meepMeep = new MeepMeep(800);
//
//            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                    .followTrajectorySequence(drive ->
//                                    drive.trajectorySequenceBuilder(left_blue)
//                                            .splineTo(new Vector2d(-46, 38), Math.toRadians(-90))
//                                            .back(6)
//                                            .strafeLeft(half_tile - square_edge)
//                                            .forward(tile + half_tile)
//                                            .splineTo(new Vector2d(-30, 8), Math.toRadians(0))
//                                            .forward(tile * 4 - half_tile)
//                                            .build()
        //                            );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}