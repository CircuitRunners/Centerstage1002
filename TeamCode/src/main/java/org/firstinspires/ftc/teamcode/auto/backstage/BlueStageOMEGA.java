package org.firstinspires.ftc.teamcode.auto.backstage;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeStackCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.presets.MoveToScoringCommand;
import org.firstinspires.ftc.teamcode.commands.presets.RetractOuttakeCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoArm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utilities.PropLocation;
import org.firstinspires.ftc.teamcode.utilities.Team;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

// Complete! :) [who needs I&R anyways?]
@Autonomous (name="BLUE BACKSTAGE 2+4")
@Config
public class BlueStageOMEGA extends CommandOpMode {
    private SampleMecanumDrive drive;

    TeamPropDetector detector;

    PropLocation locationID = PropLocation.RIGHT; // set to right by default

    private Lift lift;
    private Arm arm;
    private Claw claw;
    private ExtendoArm extendo;

    private Intake intake;

    double center_line_y = 15.5, stack_x = -56, avoidance_x_constant = 1,
            fastVelocity = 60, offsetFromBoard = 4.0;;

    Pose2d startPose = new Pose2d(10.5, 62.5,  Math.toRadians(270.00));

    Pose2d pixel_left = new Pose2d(27, 38, Math.toRadians(270.00));
    Pose2d pixel_center = new Pose2d(11.84, 33.90, Math.toRadians(270.00));
    Pose2d pixel_right = new Pose2d(27, 40, Math.toRadians(105));


    // TODO set the x values to the correct on
    Pose2d boardPosition_left = new Pose2d(52.65, 43.65 - offsetFromBoard, Math.toRadians(360));
    Pose2d boardPosition_center = new Pose2d(52.65, 39.65 - offsetFromBoard, Math.toRadians(360));
    Pose2d boardPosition_right = new Pose2d(52.65, 35  - offsetFromBoard, Math.toRadians(360));

    ParallelCommandGroup scheduledCommandGroup;

    public ParallelCommandGroup generateLeftTrajectories () {
        TrajectorySequence toPixelLeft = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(pixel_left, Math.toRadians(315)) // right pixel
                .setReversed(true)
                .splineToLinearHeading(boardPosition_left, Math.toRadians(360))
                .build();

        TrajectorySequence toStack = drive.trajectorySequenceBuilder(boardPosition_left)
                .setReversed(true)
                // move smoothly away from the board to the launch point to go across the field to stack
                .splineToConstantHeading(new Vector2d(23.55, center_line_y + 0), Math.toRadians(180.00))
                //* set the speed to be greater to zoom faster
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
                // zoom across to the pixel stack
                .splineTo(new Vector2d(stack_x + 0.0, center_line_y + 0.0), Math.toRadians(180.00))
                .build();

        TrajectorySequence toBoard = drive.trajectorySequenceBuilder(toStack.end())
                .setReversed(false)
                // zoom across the middle of the field
                .splineTo(new Vector2d(23.55, center_line_y), Math.toRadians(360.00))
                .resetVelConstraint()
                // back to the board
                .splineToLinearHeading(boardPosition_center , Math.toRadians(360.00))
                .build();

        TrajectorySequence toStackPlus6 = drive.trajectorySequenceBuilder(boardPosition_left)
                .setReversed(true)
                // move smoothly away from the board to the launch point to go across the field to stack
                .splineToConstantHeading(new Vector2d(23.55, center_line_y), Math.toRadians(180.00))
                //* set the speed to be greater to zoom faster
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
                // zoom across to the pixel stack
                .splineTo(new Vector2d(stack_x - avoidance_x_constant, center_line_y), Math.toRadians(180.00))
                .strafeRight(12)
                .strafeLeft(12)
                .setReversed(false)
                .build();

        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        // This will go to the pixel stack, then score on the board!
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toPixelLeft),
                                new SequentialCommandGroup(
                                        new WaitCommand(3100),
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.BOTTOM),
                                        new WaitCommand(1000),
                                        new InstantCommand(claw::open)
                                )
                        ),
                        // This will retract the lift, go to stack, and then intake, cumulative 2+0 up to == this
                        new ParallelCommandGroup(
                                new RetractOuttakeCommand(lift, arm, claw),
                                new TrajectorySequenceCommand(drive, toStack),
                                new SequentialCommandGroup(
                                        new WaitCommand(3000),
                                        new InstantCommand(extendo::toPixel3),
                                        new ParallelRaceGroup(
//                                                new WaitCommand(3000),
                                                new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
                                        )
                                )
                        ),
                        // Go to the board and drop, cumulative 2+2 u2==this
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toBoard),
                                new SequentialCommandGroup(
                                        new WaitCommand(3100),
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
                                        new WaitCommand(1000),
                                        new InstantCommand(claw::open)
                                )
                        ),

                        // 2+4 in progress
                        new ParallelCommandGroup(
                                new RetractOuttakeCommand(lift, arm, claw),
                                new TrajectorySequenceCommand(drive, toStack),
                                new SequentialCommandGroup(
                                        new WaitCommand(3000),
                                        new InstantCommand(extendo::down ),
                                        new ParallelRaceGroup(
//                                                new WaitCommand(3000),
                                                new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
                                        )
                                )
                        ),
                        // 2+4 complete
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toBoard),
                                new SequentialCommandGroup(
                                        new WaitCommand(3100),
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
                                        new WaitCommand(1000),
                                        new InstantCommand(claw::open)
                                )
                        ),
                        // 2+6 in progress
                        new ParallelCommandGroup(
                                new RetractOuttakeCommand(lift, arm, claw)
//                                new TrajectorySequenceCommand(drive, toStackPlus6),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(3000),
//                                        new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
//                                )
                        )
//                        // 2+6 complete
//                        new ParallelCommandGroup(
//                                new TrajectorySequenceCommand(drive, toBoard),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(3100),
//                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
//                                        new InstantCommand(claw::open)
//                                )
//                        )
                )
        );
    }



    public ParallelCommandGroup generateCenterTrajectories () {
        TrajectorySequence toPixelCenter = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(pixel_center) // center pixel
                .setReversed(true)
                .splineToLinearHeading(boardPosition_center, Math.toRadians(0))
                .build();

        TrajectorySequence toStack = drive.trajectorySequenceBuilder(boardPosition_center)
                .setReversed(true)
                // move smoothly away from the board to the launch point to go across the field to stack
                .splineToConstantHeading(new Vector2d(23.55, center_line_y + 0), Math.toRadians(180.00))
                //* set the speed to be greater to zoom faster
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
                // zoom across to the pixel stack
                .splineTo(new Vector2d(stack_x + 0.0, center_line_y + 0.0), Math.toRadians(180.00))
                .build();

        TrajectorySequence toBoard = drive.trajectorySequenceBuilder(toStack.end())
                .setReversed(false)
                // zoom across the middle of the field
                .splineTo(new Vector2d(23.55, center_line_y), Math.toRadians(360.00))
                .resetVelConstraint()
                // back to the board
                .splineToLinearHeading(boardPosition_center, Math.toRadians(360.00))
                .build();

        TrajectorySequence toStackPlus6 = drive.trajectorySequenceBuilder(boardPosition_center)
                .setReversed(true)
                // move smoothly away from the board to the launch point to go across the field to stack
                .splineToConstantHeading(new Vector2d(23.55, center_line_y), Math.toRadians(180.00))
                //* set the speed to be greater to zoom faster
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
                // zoom across to the pixel stack
                .splineTo(new Vector2d(stack_x - avoidance_x_constant, center_line_y), Math.toRadians(180.00))
                .strafeRight(12)
                .strafeLeft(12)
                .setReversed(false)
                .build();

        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        // This will go to the pixel stack, then score on the board!
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toPixelCenter),
                                new SequentialCommandGroup(
                                        new WaitCommand(3100),
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.BOTTOM),
                                        new WaitCommand(1000),
                                        new InstantCommand(claw::open)
                                )
                        ),
                        // This will retract the lift, go to stack, and then intake, cumulative 2+0 up to == this
                        new ParallelCommandGroup(
                                new RetractOuttakeCommand(lift, arm, claw),
                                new TrajectorySequenceCommand(drive, toStack),
                                new SequentialCommandGroup(
                                        new WaitCommand(3000),
                                        new InstantCommand(extendo::toPixel3),
                                        new ParallelRaceGroup(
                                                new WaitCommand(3000),
                                                new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
                                        )
                                )
                        ),
                        // Go to the board and drop, cumulative 2+2 u2==this
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toBoard),
                                new SequentialCommandGroup(
                                        new WaitCommand(3100),
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
                                        new WaitCommand(1000),
                                        new InstantCommand(claw::open)
                                )
                        ),

                        // 2+4 in progress
                        new ParallelCommandGroup(
                                new RetractOuttakeCommand(lift, arm, claw),
                                new TrajectorySequenceCommand(drive, toStack),
                                new SequentialCommandGroup(
                                        new WaitCommand(3000),
                                        new InstantCommand(extendo::down),
                                        new ParallelRaceGroup(
                                                new WaitCommand(3000),
                                                new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
                                        )
                                )
                        ),
                        // 2+4 complete
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toBoard),
                                new SequentialCommandGroup(
                                        new WaitCommand(3100),
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
                                        new WaitCommand(1000),
                                        new InstantCommand(claw::open)
                                )
                        ),
                        // 2+6 in progress
                        new ParallelCommandGroup(
                                new RetractOuttakeCommand(lift, arm, claw)
//                                new TrajectorySequenceCommand(drive, toStackPlus6),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(3000),
//                                        new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
//                                )
                        )
//                        // 2+6 complete
//                        new ParallelCommandGroup(
//                                new TrajectorySequenceCommand(drive, toBoard),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(3100),
//                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
//                                        new InstantCommand(claw::open)
//                                )
//                        )
                )
        );
    }

    public ParallelCommandGroup generateRightTrajectories () {
        TrajectorySequence toPixelRight = drive.trajectorySequenceBuilder(startPose)
            .splineTo(new Vector2d(6, 37.5), Math.toRadians(225.00)) // left
                    .setReversed(true)
                    .splineToLinearHeading(boardPosition_right, Math.toRadians(360))
                    .build();

        TrajectorySequence toStack = drive.trajectorySequenceBuilder(boardPosition_right)
                .setReversed(true)
                // move smoothly away from the board to the launch point to go across the field to stack
                .splineToConstantHeading(new Vector2d(23.55, center_line_y + 0), Math.toRadians(180.00))
                //* set the speed to be greater to zoom faster
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
                // zoom across to the pixel stack
                .splineTo(new Vector2d(stack_x + 0.0, center_line_y + 0.0), Math.toRadians(180.00))
                .build();

        TrajectorySequence toBoard = drive.trajectorySequenceBuilder(toStack.end())
                .setReversed(false)
                // zoom across the middle of the field
                .splineTo(new Vector2d(23.55, center_line_y), Math.toRadians(360.00))
                .resetVelConstraint()
                // back to the board
                .splineToLinearHeading(boardPosition_left, Math.toRadians(360.00))
                .build();

        TrajectorySequence toStackPlus6 = drive.trajectorySequenceBuilder(boardPosition_right)
                .setReversed(true)
                // move smoothly away from the board to the launch point to go across the field to stack
                .splineToConstantHeading(new Vector2d(23.55, center_line_y), Math.toRadians(180.00))
                //* set the speed to be greater to zoom faster
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
                // zoom across to the pixel stack
                .splineTo(new Vector2d(stack_x - avoidance_x_constant, center_line_y), Math.toRadians(180.00))
                .strafeRight(12)
                .strafeLeft(12)
                .setReversed(false)
                .build();

        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        // This will go to the pixel stack, then score on the board!
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toPixelRight),
                                new SequentialCommandGroup(
                                        new WaitCommand(3100),
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.BOTTOM),
                                        new WaitCommand(1000),
                                        new InstantCommand(claw::open)
                                )
                        ),
                        // This will retract the lift, go to stack, and then intake, cumulative 2+0 up to == this
                        new ParallelCommandGroup(
                                new RetractOuttakeCommand(lift, arm, claw),
                                new TrajectorySequenceCommand(drive, toStack),
                                new SequentialCommandGroup(
                                        new WaitCommand(3000),
                                        new InstantCommand(extendo::toPixel3),
                                        new ParallelRaceGroup(
                                                new WaitCommand(3000),
                                                new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
                                        )
                                )
                        ),
                        // Go to the board and drop, cumulative 2+2 u2==this
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toBoard),
                                new SequentialCommandGroup(
                                        new WaitCommand(3100),
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
                                        new WaitCommand(1000),
                                        new InstantCommand(claw::open)
                                )
                        ),

                        // 2+4 in progress
                        new ParallelCommandGroup(
                                new RetractOuttakeCommand(lift, arm, claw),
                                new TrajectorySequenceCommand(drive, toStack),
                                new SequentialCommandGroup(
                                        new WaitCommand(3000),
                                        new InstantCommand(extendo::down),
                                        new ParallelRaceGroup(
                                                new WaitCommand(3000),
                                                new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
                                        )
                                )
                        ),
                        // 2+4 complete
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toBoard),
                                new SequentialCommandGroup(
                                        new WaitCommand(3100),
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
                                        new InstantCommand(claw::open),
                                        new WaitCommand(1000),
                                        new RetractOuttakeCommand(lift,arm, claw)
                                )
                        ),
//                         2+6 in progress
                        new ParallelCommandGroup(
                                new RetractOuttakeCommand(lift, arm, claw)
//                                new TrajectorySequenceCommand(drive, toStackPlus6),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(3000),
//                                        new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
//                                )
                        )
//                       // 2+6 complete
//                        new ParallelCommandGroup(
//                                new TrajectorySequenceCommand(drive, toBoard),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(3100),
//                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
//                                        new InstantCommand(claw::open)
//                                )
//                        )
                )
        );
    }

    @Override
    public void initialize(){
        schedule(new BulkCacheCommand(hardwareMap));
        detector = new TeamPropDetector(hardwareMap, true, Team.BLUE);

        intake = new Intake(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        lift = new Lift(hardwareMap);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        extendo = new ExtendoArm(hardwareMap);

        lift.initialInitHang();

        drive.setPoseEstimate(startPose);

        ParallelCommandGroup leftPositions = generateLeftTrajectories();
        ParallelCommandGroup centerPositions = generateCenterTrajectories();
        ParallelCommandGroup rightPositions = generateRightTrajectories();

        detector.startStream();

        while(opModeInInit()){
            locationID = detector.update();
            telemetry.addData("Status", "In Init. Loading...");
            telemetry.addData("Prop", locationID.getLocation());
            telemetry.update();
            sleep(50);
        }

        detector.stopStream();

        switch (locationID) {
            case LEFT: {
                scheduledCommandGroup = leftPositions;
                break;
            }
            case MIDDLE: {
                scheduledCommandGroup = centerPositions;
                break;
            }
            case RIGHT:
            default: {
                scheduledCommandGroup = rightPositions;
            }
            break;
        }

        telemetry.addData("Status", "Loaded!");
        telemetry.update();

        schedule(scheduledCommandGroup);
    }
}
