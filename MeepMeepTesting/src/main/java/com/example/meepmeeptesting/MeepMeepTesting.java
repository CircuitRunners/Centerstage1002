package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import jdk.javadoc.internal.doclets.toolkit.taglets.UserTaglet;

public class MeepMeepTesting {


    private static double tile = 24;
    private static double half_tile = 12;
    private static double robot_len = 9;
    private static double square_edge = 1.5;


    private static Pose2d back_red = new Pose2d(12, -61.5, Math.toRadians(90));
    private static Pose2d front_red = new Pose2d(-36, -61.5, Math.toRadians(90));
    private static Pose2d back_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
    private static Pose2d front_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));
    private static Pose2d right= new Pose2d(12, -61.5, Math.toRadians(90));
    private static Pose2d left= new Pose2d(-36, -61.5, Math.toRadians(90));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double center_line_y = -11.5, stack_x = -55.5, avoidance_x_constant = 1;

        Pose2d startPose = new Pose2d(-39.5, -62.5,  Math.toRadians(90.00));

        Pose2d pixel_left = new Pose2d(-27.6, -42, Math.toRadians(90.00));
        Pose2d pixel_center = new Pose2d(-38.16, -33.90, Math.toRadians(90.00));
        Pose2d pixel_right = new Pose2d(-23, -40, Math.toRadians(105));

        Pose2d boardPosition_left = new Pose2d(50.5, -29, Math.toRadians(0));
        Pose2d boardPosition_center = new Pose2d(50.5, -35, Math.toRadians(0.00));
        Pose2d boardPosition_right = new Pose2d(50.5, -41, Math.toRadians(0));

        Pose2d purplePixel = pixel_right;
        Pose2d boardPosition = boardPosition_right;

        double fastVelocity = 60;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
               //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setConstraints(37.7594893, 37.7594893, Math.toRadians(180), Math.toRadians(180), 14.28)
                 /* .followTrajectorySequence(drive ->
                                   drive.trajectorySequenceBuilder(right_red)
                                           .forward(1.25*tile - square_edge)
                                           .back(1.1*tile)
                                           .strafeRight(tile*2)
                                           .build()
                        );*/
                       .followTrajectorySequence(drive ->
                               drive.trajectorySequenceBuilder(startPose)
                                       // pixel and backoff
//                                       .splineTo(new Vector2d(-31.90, -38.62), Math.toRadians(45.00)) // right
//                                       .splineTo(new Vector2d(-40.5, -34.5), Math.toRadians(120.00)) // left
                                       .lineToLinearHeading(new Pose2d(-35, -34, Math.toRadians(90.00)))
                                       .setReversed(true)
                                       .splineTo(new Vector2d(-37.14, -58), Math.toRadians(-90))

                                       // across the field to board
                                       .setReversed(false)
                                       .lineTo(new Vector2d(32.16, -58))
                                       .splineToLinearHeading(new Pose2d(51.01, -40.24, Math.toRadians(0.00)), Math.toRadians(0.00))

                                       // from board to first stack
                                       .setReversed(true)
                                       .splineToConstantHeading(new Vector2d(32.16, -58), Math.toRadians(180))
                                       .lineTo(new Vector2d(-37.14, -58))
                                       .splineToConstantHeading(new Vector2d(-60, -37), Math.toRadians(180))

                                       // first stack to board
                                       .setReversed(false)
                                       .splineToConstantHeading(new Vector2d(-37.14, -58), Math.toRadians(0))
                                       .lineTo(new Vector2d(32.16, -58))
                                       .splineToConstantHeading(new Vector2d(51.01, -40.24), Math.toRadians(0))

                                       // board to second stack
                                       .setReversed(true)
                                       .splineToConstantHeading(new Vector2d(32.16, -58), Math.toRadians(180))
                                       .lineTo(new Vector2d(-37.14, -58))
                                       .splineToConstantHeading(new Vector2d(-60, -37), Math.toRadians(180))

                                       // stack to board
                                       .setReversed(false)
                                       .splineToConstantHeading(new Vector2d(-37.14, -58), Math.toRadians(0))
                                       .lineTo(new Vector2d(32.16, -58))
                                       .splineToConstantHeading(new Vector2d(51.01, -40.24), Math.toRadians(0))


//                                       .lineTo(new Vector2d(34.59, -58.68))
//
//
////                                       .setReversed(true)
////                                       .splineTo(new Vector2d(-30, -37), Math.toRadians(45))
//////                                       .splineToSplineHeading(purplePixel, Math.toRadians(45)) // right pixel
//////                                       .splineTo(new Vector2d(-42, -37.5), Math.toRadians(135))
////
//////                                       .splineTo(new Vector2d(-30, -56), Math.toRadians(90))
//////                                       .setReversed(false)
////                                       .splineTo(new Vector2d(-35, -58), Math.toRadians(90))
////                                       .lineTo(new Vector2d(37, -58))
//
//                                       .splineToLinearHeading(boardPosition, Math.toRadians(0))
//
//                                       .setReversed(true)
//                                       // move smoothly away from the board to the launch point to go across the field to stack
//                                       .splineToConstantHeading(new Vector2d(23.55, center_line_y), Math.toRadians(180.00))
//                                       //* set the speed to be greater to zoom faster
//                                       .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, 14.28))
//                                       // zoom across to the pixel stack
//                                       .splineTo(new Vector2d(stack_x, center_line_y), Math.toRadians(180.00))
//
//
//                                       .setReversed(false)
//                                       // zoom across the middle of the field
//                                       .splineTo(new Vector2d(23.55, center_line_y), Math.toRadians(0.00))
//                                       .resetVelConstraint()
//                                       // back to the board
//                                       .splineToLinearHeading(boardPosition, Math.toRadians(0.00))
//
//                                       .setReversed(true)
//                                       // move smoothly away from the board to the launch point to go across the field to stack
//                                       .splineToConstantHeading(new Vector2d(23.55, center_line_y), Math.toRadians(180.00))
//                                       //* set the speed to be greater to zoom faster
//                                       .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, 14.28))
//                                       // zoom across to the pixel stack
//                                       .splineTo(new Vector2d(stack_x, center_line_y), Math.toRadians(180.00))
//
//
//                                       .setReversed(false)
//                                       // zoom across the middle of the field
//                                       .splineTo(new Vector2d(23.55, center_line_y), Math.toRadians(0.00))
//                                       .resetVelConstraint()
//                                       // back to the board
//                                       .splineToLinearHeading(boardPosition, Math.toRadians(0.00))
//
//                                       .setReversed(true)
//                                       // move smoothly away from the board to the launch point to go across the field to stack
//                                       .splineToConstantHeading(new Vector2d(23.55, center_line_y), Math.toRadians(180.00))
//                                       //* set the speed to be greater to zoom faster
//                                       .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, 14.28))
//                                       // zoom across to the pixel stack
//                                       .splineTo(new Vector2d(stack_x - avoidance_x_constant, center_line_y), Math.toRadians(180.00))
//                                       .strafeRight(12)
//                                       .strafeLeft(12)
//                                       .setReversed(false)
//
//                                       // zoom across the middle of the field
//                                       .splineTo(new Vector2d(23.55, center_line_y), Math.toRadians(0.00))
//                                       .resetVelConstraint()
//                                       // back to the board
//                                       .splineToLinearHeading(boardPosition, Math.toRadians(0.00))

//                                        // come back from the board a tiny bit
//                                       .lineToConstantHeading(new Vector2d(46.54, -27.71))
//                                       // move up to the park zone and turn so that dillans forward is close to forward
//                                       .lineToSplineHeading(new Pose2d(46.36, -12.82, Math.toRadians(90)))



                                       .build()













                               );//purple to left & park
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