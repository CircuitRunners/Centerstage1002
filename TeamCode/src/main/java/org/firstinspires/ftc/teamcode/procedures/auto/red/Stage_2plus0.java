//package org.firstinspires.ftc.teamcode.procedures.auto.red;
//
//import static org.firstinspires.ftc.teamcode.controllers.auto.roadrunner.DriveConstants.TRACK_WIDTH;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands.TrajectorySequenceCommand;
//import org.firstinspires.ftc.teamcode.controllers.auto.roadrunner.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.controllers.auto.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.controllers.commands.presets.MoveToScoringCommand;
//import org.firstinspires.ftc.teamcode.controllers.commands.presets.RetractOuttakeCommand;
//import org.firstinspires.ftc.teamcode.controllers.common.utilities.PropLocation;
//import org.firstinspires.ftc.teamcode.controllers.common.utilities.Team;
//import org.firstinspires.ftc.teamcode.controllers.subsytems.Arm;
//import org.firstinspires.ftc.teamcode.controllers.subsytems.Claw;
//import org.firstinspires.ftc.teamcode.controllers.subsytems.ExtendoArm;
//import org.firstinspires.ftc.teamcode.controllers.subsytems.Intake;
//import org.firstinspires.ftc.teamcode.controllers.subsytems.Lift;
//import org.firstinspires.ftc.teamcode.controllers.vision.TeamPropDetector;
//import org.firstinspires.ftc.teamcode.procedures.AutoBase;
//
//// Complete! :) [who needs I&R anyways?]
//@Autonomous (name="BLUE BACKSTAGE 2+0")
//@Config
//public class Stage_2plus0 extends AutoBase {
//    double center_line_y = 15.5, stack_x = -56, avoidance_x_constant = 1,
//            fastVelocity = 60, offsetFromBoard = 4.0;;
//
//    Pose2d startPose = new Pose2d(10.5, 62.5,  Math.toRadians(270.00));
//
//    Pose2d pixel_left = new Pose2d(23.5, 38, Math.toRadians(270.00));
//    Pose2d pixel_center = new Pose2d(11.84, 33.90, Math.toRadians(270.00));
//    Pose2d pixel_right = new Pose2d(27, 40, Math.toRadians(105));
//
//    private ParallelCommandGroup leftSequence, rightSequence, centerSequence;
//
//    // TODO set the x values to the correct on
//    Pose2d boardPosition_left = new Pose2d(52.65, 45.65 - offsetFromBoard, Math.toRadians(360));
//    Pose2d boardPosition_center = new Pose2d(52.65, 39.65 - offsetFromBoard, Math.toRadians(360));
//    Pose2d boardPosition_right = new Pose2d(52.65, 35.5  - offsetFromBoard, Math.toRadians(360));
//
//    public ParallelCommandGroup generateLeftTrajectories () {
//        TrajectorySequence toPixelLeft = robot.drive.trajectorySequenceBuilder(startPose)
//                .splineToSplineHeading(pixel_left, Math.toRadians(315)) // right pixel
//                .setReversed(true)
//                .splineToLinearHeading(boardPosition_left, Math.toRadians(360))
//                .build();
//
//        TrajectorySequence toStack = robot.drive.trajectorySequenceBuilder(boardPosition_left)
//                .setReversed(true)
//                // move smoothly away from the board to the launch point to go across the field to stack
//                .lineTo(new Vector2d(48.5, 12))
//                .build();
//
//        TrajectorySequence toBoard = robot.drive.trajectorySequenceBuilder(toStack.end())
//                .setReversed(false)
//                // zoom across the middle of the field
//                .splineTo(new Vector2d(23.55, center_line_y), Math.toRadians(360.00))
//                .resetVelConstraint()
//                // back to the board
//                .splineToLinearHeading(boardPosition_center , Math.toRadians(360.00))
//                .build();
//
//        TrajectorySequence toStackPlus6 = robot.drive.trajectorySequenceBuilder(boardPosition_left)
//                .setReversed(true)
//                // move smoothly away from the board to the launch point to go across the field to stack
//                .splineToConstantHeading(new Vector2d(23.55, center_line_y), Math.toRadians(180.00))
//                //* set the speed to be greater to zoom faster
//                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
//                // zoom across to the pixel stack
//                .splineTo(new Vector2d(stack_x - avoidance_x_constant, center_line_y), Math.toRadians(180.00))
//                .strafeRight(12)
//                .strafeLeft(12)
//                .setReversed(false)
//                .build();
//
//        return new ParallelCommandGroup(
//                new SequentialCommandGroup(
//                        // This will go to the pixel stack, then score on the board!
//                        new ParallelCommandGroup(
//                                new TrajectorySequenceCommand(robot.drive, toPixelLeft),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(3100),
//                                        new MoveToScoringCommand(robot.lift, robot.arm, robot.claw, MoveToScoringCommand.Presets.BOTTOM),
//                                        new WaitCommand(1000),
//                                        new InstantCommand(robot.claw::open)
//                                )
//                        ),
//                        // This will retract the lift, go to stack, and then intake, cumulative 2+0 up to == this
//                        new ParallelCommandGroup(
//                                new RetractOuttakeCommand(robot.lift, robot.arm, robot.claw),
//                                new TrajectorySequenceCommand(robot.drive, toStack)
//                        )
////                        // Go to the board and drop, cumulative 2+2 u2==this
////                        new ParallelCommandGroup(
////                                new TrajectorySequenceCommand(drive, toBoard),
////                                new SequentialCommandGroup(
////                                        new WaitCommand(3100),
////                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
////                                        new WaitCommand(1000),
////                                        new InstantCommand(claw::open)
////                                )
////                        ),
////
////                        // 2+4 in progress
////                        new ParallelCommandGroup(
////                                new RetractOuttakeCommand(lift, arm, claw),
////                                new TrajectorySequenceCommand(drive, toStack),
////                                new SequentialCommandGroup(
////                                        new WaitCommand(3000),
////                                        new InstantCommand(extendo::down ),
////                                        new ParallelRaceGroup(
//////                                                new WaitCommand(3000),
////                                                new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
////                                        )
////                                )
////                        ),
////                        // 2+4 complete
////                        new ParallelCommandGroup(
////                                new TrajectorySequenceCommand(drive, toBoard),
////                                new SequentialCommandGroup(
////                                        new WaitCommand(3100),
////                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
////                                        new WaitCommand(1000),
////                                        new InstantCommand(claw::open)
////                                )
////                        ),
////                        // 2+6 in progress
////                        new ParallelCommandGroup(
////                                new RetractOuttakeCommand(lift, arm, claw)
//////                                new TrajectorySequenceCommand(drive, toStackPlus6),
//////                                new SequentialCommandGroup(
//////                                        new WaitCommand(3000),
//////                                        new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
//////                                )
////                        )
//////                        // 2+6 complete
//////                        new ParallelCommandGroup(
//////                                new TrajectorySequenceCommand(drive, toBoard),
//////                                new SequentialCommandGroup(
//////                                        new WaitCommand(3100),
//////                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
//////                                        new InstantCommand(claw::open)
//////                                )
//////                        )
//                )
//        );
//    }
//
//
//
//    public ParallelCommandGroup generateCenterTrajectories () {
//        TrajectorySequence toPixelCenter = robot.drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(pixel_center) // center pixel
//                .setReversed(true)
//                .splineToLinearHeading(boardPosition_center, Math.toRadians(0))
//                .build();
//
//        TrajectorySequence toStack = robot.drive.trajectorySequenceBuilder(boardPosition_center)
//                .setReversed(true)
//                .lineTo(new Vector2d(48.5, 12))
//                .build();
//
//        TrajectorySequence toBoard = robot.drive.trajectorySequenceBuilder(toStack.end())
//                .setReversed(false)
//                // zoom across the middle of the field
//                .splineTo(new Vector2d(23.55, center_line_y), Math.toRadians(360.00))
//                .resetVelConstraint()
//                // back to the board
//                .splineToLinearHeading(boardPosition_center, Math.toRadians(360.00))
//                .build();
//
//        TrajectorySequence toStackPlus6 = robot.drive.trajectorySequenceBuilder(boardPosition_center)
//                .setReversed(true)
//                // move smoothly away from the board to the launch point to go across the field to stack
//                .splineToConstantHeading(new Vector2d(23.55, center_line_y), Math.toRadians(180.00))
//                //* set the speed to be greater to zoom faster
//                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
//                // zoom across to the pixel stack
//                .splineTo(new Vector2d(stack_x - avoidance_x_constant, center_line_y), Math.toRadians(180.00))
//                .strafeRight(12)
//                .strafeLeft(12)
//                .setReversed(false)
//                .build();
//
//        return new ParallelCommandGroup(
//                new SequentialCommandGroup(
//                        // This will go to the pixel stack, then score on the board!
//                        new ParallelCommandGroup(
//                                new TrajectorySequenceCommand(robot.drive, toPixelCenter),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(3100),
//                                        new MoveToScoringCommand(robot.lift, robot.arm, robot.claw, MoveToScoringCommand.Presets.BOTTOM),
//                                        new WaitCommand(1000),
//                                        new InstantCommand(robot.claw::open)
//                                )
//                        ),
//                        // This will retract the lift, go to stack, and then intake, cumulative 2+0 up to == this
//                        new ParallelCommandGroup(
//                                new RetractOuttakeCommand(robot.lift, robot.arm, robot.claw)
////                                new TrajectorySequenceCommand(drive, toStack)
////                                new SequentialCommandGroup(
////                                        new WaitCommand(3000),
////                                        new InstantCommand(extendo::toPixel3),
////                                        new ParallelRaceGroup(
////                                                new WaitCommand(3000),
////                                                new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
////                                        )
////                                )
//                        )
//                )
//        );
//    }
//
//    public ParallelCommandGroup generateRightTrajectories () {
//        TrajectorySequence toPixelRight = robot.drive.trajectorySequenceBuilder(startPose)
//                .splineTo(new Vector2d(6, 37.5), Math.toRadians(225.00)) // left
//                .setReversed(true)
//                .splineToLinearHeading(boardPosition_right, Math.toRadians(360))
//                .build();
//
//        TrajectorySequence toStack = robot.drive.trajectorySequenceBuilder(boardPosition_right)
//                .setReversed(true)
//                .lineTo(new Vector2d(48.5, 12))
//                .build();
//
//        TrajectorySequence toBoard = robot.drive.trajectorySequenceBuilder(toStack.end())
//                .setReversed(false)
//                // zoom across the middle of the field
//                .splineTo(new Vector2d(23.55, center_line_y), Math.toRadians(360.00))
//                .resetVelConstraint()
//                // back to the board
//                .splineToLinearHeading(boardPosition_left, Math.toRadians(360.00))
//                .build();
//
//        TrajectorySequence toStackPlus6 = robot.drive.trajectorySequenceBuilder(boardPosition_right)
//                .setReversed(true)
//                // move smoothly away from the board to the launch point to go across the field to stack
//                .splineToConstantHeading(new Vector2d(23.55, center_line_y), Math.toRadians(180.00))
//                //* set the speed to be greater to zoom faster
//                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
//                // zoom across to the pixel stack
//                .splineTo(new Vector2d(stack_x - avoidance_x_constant, center_line_y), Math.toRadians(180.00))
//                .strafeRight(12)
//                .strafeLeft(12)
//                .setReversed(false)
//                .build();
//
//        return new ParallelCommandGroup(
//                new SequentialCommandGroup(
//                        // This will go to the pixel stack, then score on the board!
//                        new ParallelCommandGroup(
//                                new TrajectorySequenceCommand(robot.drive, toPixelRight),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(3100),
//                                        new MoveToScoringCommand(robot.lift, robot.arm, robot.claw, MoveToScoringCommand.Presets.BOTTOM),
//                                        new WaitCommand(1000),
//                                        new InstantCommand(robot.claw::open)
//                                )
//                        ),
//                        // This will retract the lift, go to stack, and then intake, cumulative 2+0 up to == this
//                        new ParallelCommandGroup(
//                                new RetractOuttakeCommand(robot.lift, robot.arm, robot.claw),
//                                new TrajectorySequenceCommand(robot.drive, toStack)
////                                new SequentialCommandGroup(
////                                        new WaitCommand(3000),
////                                        new InstantCommand(extendo::toPixel3),
////                                        new ParallelRaceGroup(
////                                                new WaitCommand(3000),
////                                                new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
////                                        )
////                                )
//                        )
////                        // Go to the board and drop, cumulative 2+2 u2==this
////                        new ParallelCommandGroup(
////                                new TrajectorySequenceCommand(drive, toBoard),
////                                new SequentialCommandGroup(
////                                        new WaitCommand(3100),
////                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
////                                        new WaitCommand(1000),
////                                        new InstantCommand(claw::open)
////                                )
////                        ),
////
////                        // 2+4 in progress
////                        new ParallelCommandGroup(
////                                new RetractOuttakeCommand(lift, arm, claw),
////                                new TrajectorySequenceCommand(drive, toStack),
////                                new SequentialCommandGroup(
////                                        new WaitCommand(3000),
////                                        new InstantCommand(extendo::down),
////                                        new ParallelRaceGroup(
////                                                new WaitCommand(3000),
////                                                new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
////                                        )
////                                )
////                        ),
////                        // 2+4 complete
////                        new ParallelCommandGroup(
////                                new TrajectorySequenceCommand(drive, toBoard),
////                                new SequentialCommandGroup(
////                                        new WaitCommand(3100),
////                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
////                                        new InstantCommand(claw::open),
////                                        new WaitCommand(1000),
////                                        new RetractOuttakeCommand(lift,arm, claw)
////                                )
////                        ),
//////                         2+6 in progress
////                        new ParallelCommandGroup(
////                                new RetractOuttakeCommand(lift, arm, claw)
//////                                new TrajectorySequenceCommand(drive, toStackPlus6),
//////                                new SequentialCommandGroup(
//////                                        new WaitCommand(3000),
//////                                        new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
//////                                )
////                        )
//////                       // 2+6 complete
//////                        new ParallelCommandGroup(
//////                                new TrajectorySequenceCommand(drive, toBoard),
//////                                new SequentialCommandGroup(
//////                                        new WaitCommand(3100),
//////                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
//////                                        new InstantCommand(claw::open)
//////                                )
//////                        )
//                )
//        );
//    }
//
//    @Override
//    public void build () {
//        leftSequence = generateLeftTrajectories();
//        centerSequence = generateCenterTrajectories();
//        rightSequence = generateRightTrajectories();
//    }
//
//    @Override
//    public void runAuto(){
//        telemetry.addLine("go");
//        telemetry.update();
//        switch (PropLocation.LEFT) {
//            case LEFT: {
//                schedule(leftSequence);
//                break;
//            }
//            case MIDDLE: {
//                schedule(centerSequence);
//                break;
//            }
//            case RIGHT:
//            default: {
//                schedule(rightSequence);
//            }
//            break;
//        }
//    }
//}
