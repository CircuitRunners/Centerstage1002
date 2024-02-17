package org.firstinspires.ftc.teamcode.auto.audience;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
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
@Autonomous (name="RED AUDIENCE OMEGA")
@Config
public class RedAudienceOMEGA extends CommandOpMode {
    private SampleMecanumDrive drive;

    TeamPropDetector detector;

    PropLocation locationID = PropLocation.RIGHT; // set to right by default

    private Lift lift;
    private Arm arm;
    private Claw claw;
    private ExtendoArm extendo;

    private Intake intake;

    double center_line_y = -12.5, stack_x = -53.5, avoidance_x_constant = 1,
            fastVelocity = 60, offsetFromBoard = 4.0;;

    Pose2d startPose = new Pose2d(10.5, -62.5,  Math.toRadians(90.00));

    Pose2d pixel_left = new Pose2d(22.4, -42, Math.toRadians(90.00));
    Pose2d pixel_center = new Pose2d(11.84, -33.90, Math.toRadians(90.00));
    Pose2d pixel_right = new Pose2d(27, -40, Math.toRadians(105));


    // TODO set the x values to the correct on
    Pose2d boardPosition_left = new Pose2d(52, -32 - offsetFromBoard, Math.toRadians(0));
    Pose2d boardPosition_center = new Pose2d(50.5, -35 - offsetFromBoard, Math.toRadians(0));
    Pose2d boardPosition_right = new Pose2d(50.5, -38 - offsetFromBoard, Math.toRadians(0));

    ParallelCommandGroup scheduledCommandGroup;

    public ParallelCommandGroup generateTrajectories (TrajectorySequence toPixel, Pose2d boardPosition) {

        TrajectorySequence toBoardFirstPixel = drive.trajectorySequenceBuilder(toPixel.end())
                // across the field to board
                .setReversed(false)
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
                .lineTo(new Vector2d(32.16, -58))
                .resetVelConstraint()
                .splineToLinearHeading(boardPosition, Math.toRadians(0.00))
                .build();

        TrajectorySequence toStack = drive.trajectorySequenceBuilder(toBoardFirstPixel.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(32.16, -58), Math.toRadians(180))
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
                .lineTo(new Vector2d(-37.14, -58))
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(-60, -37), Math.toRadians(180))
                .build();

        TrajectorySequence toBoard = drive.trajectorySequenceBuilder(toStack.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-37.14, -58), Math.toRadians(0))
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
                .lineTo(new Vector2d(32.16, -58))
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(boardPosition.getX(), boardPosition.getY()), Math.toRadians(0))
                .build();

        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(toBoard.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(32.16, -58), Math.toRadians(180))
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
                .lineTo(new Vector2d(-37.14, -58))
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(-60, -37), Math.toRadians(180))
                .build();

        TrajectorySequence toBoard2 = drive.trajectorySequenceBuilder(toStack.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-37.14, -58), Math.toRadians(0))
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
                .lineTo(new Vector2d(32.16, -58))
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(boardPosition.getX(), boardPosition.getY()), Math.toRadians(0))
                .build();

        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new TrajectorySequenceCommand(drive, toPixel),
                        // This will go to the pixel stack, then score on the board!
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toBoardFirstPixel),
                                new SequentialCommandGroup(
                                        new WaitCommand(3800),
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.BOTTOM),
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
                                        new WaitCommand(3800),
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
                                        new InstantCommand(claw::open)
                                )
                        ),

                        // 2+4 in progress
                        new ParallelCommandGroup(
                                new RetractOuttakeCommand(lift, arm, claw),
                                new TrajectorySequenceCommand(drive, toStack2),
                                new SequentialCommandGroup(
                                        new WaitCommand(3000),
                                        new InstantCommand(extendo::toPixel3),
                                        new ParallelRaceGroup(
                                                new WaitCommand(3000),
                                                new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST)
                                        )
                                )
                        ),
                        // 2+4 complete
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, toBoard2),
                                new SequentialCommandGroup(
                                        new WaitCommand(3800),
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
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
                )
        );
    }

    @Override
    public void initialize(){
        schedule(new BulkCacheCommand(hardwareMap));
        detector = new TeamPropDetector(hardwareMap, true, Team.RED);

        intake = new Intake(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        lift = new Lift(hardwareMap);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        extendo = new ExtendoArm(hardwareMap);

        lift.initialInitHang();

        drive.setPoseEstimate(startPose);

        ParallelCommandGroup leftPositions = generateTrajectories(
                drive.trajectorySequenceBuilder(startPose)
                        .splineTo(new Vector2d(-40.5, -34.5), Math.toRadians(120.00))
                        .setReversed(true)
                        .splineTo(new Vector2d(-37.14, -58), Math.toRadians(-90))
                        .build(),
                boardPosition_left
        );
        ParallelCommandGroup centerPositions = generateTrajectories(
                drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(-35, -34, Math.toRadians(90.00)))
                        .setReversed(true)
                        .splineTo(new Vector2d(-37.14, -58), Math.toRadians(-90))
                        .build(),
                boardPosition_center
        );
        ParallelCommandGroup rightPositions = generateTrajectories(
                drive.trajectorySequenceBuilder(startPose)
                        .splineTo(new Vector2d(-31.90, -38.62), Math.toRadians(45.00))
                        .setReversed(true)
                        .splineTo(new Vector2d(-37.14, -58), Math.toRadians(-90))
                        .build(),
                boardPosition_right
        );

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