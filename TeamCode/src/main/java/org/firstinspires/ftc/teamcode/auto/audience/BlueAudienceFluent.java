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
@Autonomous (name="BLUE AUDIENCE 2+1")
@Config
public class BlueAudienceFluent extends CommandOpMode {
    private SampleMecanumDrive drive;

    TeamPropDetector detector;

    PropLocation locationID = PropLocation.RIGHT; // set to right by default

    private Lift lift;
    private Arm arm;
    private Claw claw;
    private ExtendoArm extendo;

    private Intake intake;

    double center_line_y = 60.5, stack_x = -53.5, avoidance_x_constant = 1,
            fastVelocity = 60, offsetFromBoard = 4.0;;

    Pose2d startPose = new Pose2d(-38.39, 63.28, Math.toRadians(270.00));

    Pose2d pixel_left = new Pose2d(-48, 34.90, Math.toRadians(270.00));
    Pose2d pixel_center = new Pose2d(-38.39, 33.90, Math.toRadians(270.00));
    Pose2d pixel_right = new Pose2d(-48, 34.90, Math.toRadians(270));


    // TODO set the x values to the correct on
    Pose2d boardPosition_left = new Pose2d(52, 38 - offsetFromBoard, Math.toRadians(0));
    Pose2d boardPosition_center = new Pose2d(50.5, 35 - offsetFromBoard, Math.toRadians(0));
    Pose2d boardPosition_right = new Pose2d(50.5, 32 - offsetFromBoard, Math.toRadians(0));

    ParallelCommandGroup scheduledCommandGroup;

    public ParallelCommandGroup generateLeftTrajectories () {
        TrajectorySequence toPixelCenter = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-31,37.5),Math.toRadians(315))
                .lineTo(new Vector2d(-38.39, 36.90))// center pixel
                .setReversed(true)
                .splineTo(new Vector2d(stack_x, 36.69), Math.toRadians(180.00))
                .build();


        TrajectorySequence toBackboard = drive.trajectorySequenceBuilder(toPixelCenter.end())
                // move smoothly away from the board to the launch point to go across the field to stack
                .lineTo(new Vector2d(stack_x,center_line_y))
                // zoom across to the pixel stack
                .setReversed(false)
                .splineTo(new Vector2d(21.71, center_line_y + 0.0), Math.toRadians(360))
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
                .splineToLinearHeading(boardPosition_left, Math.toRadians(0))
                .resetVelConstraint()
                .build();

        TrajectorySequence Park = drive.trajectorySequenceBuilder(toBackboard.end())
                .setReversed(false)
                // zoom across the middle of the field
                .lineTo(new Vector2d(46.5, -35 - offsetFromBoard))
                .lineTo(new Vector2d(46.5,center_line_y))
                // back to the board

                .build();


        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        // This will go to the pixel stack, then score on the board!
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toPixelCenter),
                                new InstantCommand(claw:: open),
                                new SequentialCommandGroup(
                                        new WaitCommand(3000),
                                        new InstantCommand(extendo::toPixel3),
                                        new ParallelRaceGroup(
                                                new WaitCommand(6000),
                                                new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
                                        )
                                )
                        ),
//                        // This will retract the lift, go to stack, and then intake, cumulative 2+0 up to == this
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toBackboard),
                                new SequentialCommandGroup(
                                        new WaitCommand(4000),
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
                                        new InstantCommand(claw:: open)

                                )
                        ),
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, Park),
                                new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new RetractOuttakeCommand(lift,arm,claw)
                                )
                        )
//
                )
        );
    }



    public ParallelCommandGroup generateCenterTrajectories () {
        TrajectorySequence toPixelCenter = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(pixel_center)
                .lineTo(new Vector2d(-38.39, -36.90))// center pixel
                .setReversed(true)
                .splineTo(new Vector2d(stack_x, -36.69), Math.toRadians(180.00))
                .build();


        TrajectorySequence toBackboard = drive.trajectorySequenceBuilder(toPixelCenter.end())
                // move smoothly away from the board to the launch point to go across the field to stack
                .lineTo(new Vector2d(stack_x,center_line_y))
                // zoom across to the pixel stack
                .setReversed(false)
                .splineTo(new Vector2d(21.71, center_line_y + 0.0), Math.toRadians(360))
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
                .splineToLinearHeading(boardPosition_center, Math.toRadians(0))
                .resetVelConstraint()
                .build();

        TrajectorySequence Park = drive.trajectorySequenceBuilder(toBackboard.end())
                .setReversed(false)
                // zoom across the middle of the field
                .lineTo(new Vector2d(46.5, -35 - offsetFromBoard))
                .lineTo(new Vector2d(46.5,center_line_y))
                // back to the board

                .build();


        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        // This will go to the pixel stack, then score on the board!
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toPixelCenter),
                                new InstantCommand(claw:: open),
                                new SequentialCommandGroup(
                                        new WaitCommand(3000),
                                        new InstantCommand(extendo::toPixel3),
                                        new ParallelRaceGroup(
                                                new WaitCommand(6000),
                                                new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
                                        )
                                )
                        ),
//                        // This will retract the lift, go to stack, and then intake, cumulative 2+0 up to == this
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toBackboard),
                                new SequentialCommandGroup(
                                        new WaitCommand(4000),
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
                                        new InstantCommand(claw:: open)

                                )
                        ),
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, Park),
                                new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new RetractOuttakeCommand(lift,arm,claw)
                                )
                        )
//
                )
        );
    }

    public ParallelCommandGroup generateRightTrajectories () {
        TrajectorySequence toPixelCenter = drive.trajectorySequenceBuilder(startPose)
            .lineToLinearHeading(pixel_right)
                    .lineTo(new Vector2d(-48, 40.90))
                    .lineTo(new Vector2d(-38.39, 36.90))// center pixel
                    .setReversed(true)
                    .splineTo(new Vector2d(stack_x, 36.69), Math.toRadians(180.00))
                    .build();



        TrajectorySequence toBackboard = drive.trajectorySequenceBuilder(toPixelCenter.end())
                // move smoothly away from the board to the launch point to go across the field to stack
                .lineTo(new Vector2d(stack_x,center_line_y))
                // zoom across to the pixel stack
                .setReversed(false)
                .splineTo(new Vector2d(21.71, center_line_y + 0.0), Math.toRadians(360))
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
                .splineToLinearHeading(boardPosition_right, Math.toRadians(0))
                .resetVelConstraint()
                .build();

        TrajectorySequence Park = drive.trajectorySequenceBuilder(toBackboard.end())
                .setReversed(false)
                // zoom across the middle of the field
                .lineTo(new Vector2d(46.5, -35 - offsetFromBoard))
                .lineTo(new Vector2d(46.5,center_line_y))
                // back to the board

                .build();


        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        // This will go to the pixel stack, then score on the board!
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toPixelCenter),
                                new InstantCommand(claw:: open),
                                new SequentialCommandGroup(
                                        new WaitCommand(3000),
                                        new InstantCommand(extendo::toPixel3),
                                        new ParallelRaceGroup(
                                                new WaitCommand(6000),
                                                new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
                                        )
                                )
                        ),
//                        // This will retract the lift, go to stack, and then intake, cumulative 2+0 up to == this
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toBackboard),
                                new SequentialCommandGroup(
                                        new WaitCommand(4000),
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
                                        new InstantCommand(claw:: open)

                                )
                        ),
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, Park),
                                new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new RetractOuttakeCommand(lift,arm,claw)
                                )
                        )
//
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