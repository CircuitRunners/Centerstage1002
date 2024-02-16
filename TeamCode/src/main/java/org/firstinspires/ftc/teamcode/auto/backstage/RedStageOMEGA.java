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
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
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
@Autonomous (name="RED BACKSTAGE OMEGA")
@Config
public class RedStageOMEGA extends CommandOpMode {
    private SampleMecanumDrive drive;

    TeamPropDetector detector;

    PropLocation locationID = PropLocation.RIGHT; // set to center by default
    PropLocation lastLocation = null; // set to center by default

    private Lift lift;
    private Arm arm;
    private Claw claw;
    private ExtendoArm extendo;

    private Intake intake;

    private Integer iteration = 0;


//    private Pose2d startPose = Pose2dMapped(9.00, -61.50, Math.toRadians(90.00));
//    private Pose2d startPose = Pose2dMapped(10.25, -63,  Math.toRadians(90.00));
//    public Pose2d startPose = Pose2dMapped(10.5, -62.5,  Math.toRadians(90.00));

    double center_line_y = -12.5, stack_x = -55.5, avoidance_x_constant = 1,
            fastVelocity = 60, offsetFromBoard = 4.0;;

    Pose2d startPose = new Pose2d(10.5, -62.5,  Math.toRadians(90.00));

    Pose2d pixel_left = new Pose2d(22.4, -42, Math.toRadians(90.00));
    Pose2d pixel_center = new Pose2d(11.84, -33.90, Math.toRadians(90.00));
    Pose2d pixel_right = new Pose2d(27, -40, Math.toRadians(105));


    Pose2d boardPosition_left = new Pose2d(50.5, -32 - offsetFromBoard, Math.toRadians(0));
    Pose2d boardPosition_center = new Pose2d(50.5, -35 - offsetFromBoard, Math.toRadians(0.00));
    Pose2d boardPosition_right = new Pose2d(50.5, -38 - offsetFromBoard, Math.toRadians(0));

    TrajectorySequence toPixel, toBoard, toStack, toStackThenBoard;

    public void generateTrajectories (PropLocation propLocation) {
        Pose2d boardPosition;
        switch (propLocation) {
            case LEFT: {
                boardPosition = boardPosition_left;
                break;
            }
            case MIDDLE: {
                boardPosition = boardPosition_center;
                break;
            }
            case RIGHT:
            default: {
                boardPosition = boardPosition_right;
                break;
            }
        }

        toStack = drive.trajectorySequenceBuilder(boardPosition)
                .setReversed(true)
                // move smoothly away from the board to the launch point to go across the field to stack
                .splineToConstantHeading(new Vector2d(23.55, center_line_y), Math.toRadians(180.00))
                //* set the speed to be greater to zoom faster
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, TRACK_WIDTH))
                // zoom across to the pixel stack
                .splineTo(new Vector2d(stack_x, center_line_y), Math.toRadians(180.00))
                .build();

        toBoard = drive.trajectorySequenceBuilder(toStack.end())
                .setReversed(false)
                // zoom across the middle of the field
                .splineTo(new Vector2d(23.55, center_line_y), Math.toRadians(0.00))
                .resetVelConstraint()
                // back to the board
                .splineToLinearHeading(boardPosition, Math.toRadians(0.00))
                .build();

        toStackThenBoard = drive.trajectorySequenceBuilder(boardPosition)
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
        // End
    }

    public ParallelCommandGroup runAtTime (long ms, Command command) {
        return new ParallelCommandGroup(
            new WaitCommand(ms),
            command
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

        TrajectorySequence toPixelLeft = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(8, -37.5), Math.toRadians(135.00)) // left
                .setReversed(true)
                .splineToLinearHeading(boardPosition_left, Math.toRadians(0))
                .build();

        TrajectorySequence toPixelCenter = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(pixel_center) // center pixel
                .setReversed(true)
                .splineToLinearHeading(boardPosition_center, Math.toRadians(0))
                .build();

        TrajectorySequence toPixelRight = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(pixel_right, Math.toRadians(45)) // right pixel
                .setReversed(true)
                .splineToLinearHeading(boardPosition_right, Math.toRadians(0))
                .build();

        detector.startStream();
//
        while(opModeInInit()){
            locationID = detector.update();
            telemetry.addLine("Ready for start!");
            telemetry.addData("Prop", locationID.getLocation());
            telemetry.update();

            if (lastLocation != locationID) {
                switch (locationID) {
                    case LEFT: {
                        toPixel = toPixelLeft;
                        break;
                    }
                    case MIDDLE: {
                        toPixel = toPixelCenter;
                        break;
                    }
                    case RIGHT: {
                        toPixel = toPixelRight;
                        break;

                    }
                }

                generateTrajectories(locationID);
            }

            sleep(50);

        }
//
        detector.stopStream();

        schedule(
                new SequentialCommandGroup(
                        new TrajectorySequenceCommand(drive, toPixel),
                        new TrajectorySequenceCommand(drive, toBoard),
                        new TrajectorySequenceCommand(drive, toStack),
                        new TrajectorySequenceCommand(drive, toBoard),
                        new TrajectorySequenceCommand(drive, toStack),
                        new TrajectorySequenceCommand(drive, toBoard),
                        new TrajectorySequenceCommand(drive, toStackThenBoard)
                ),
                new SequentialCommandGroup(
                        runAtTime(2800, new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT)),
                        runAtTime(3000, new InstantCommand(claw::open)),
                        runAtTime(4270, new SequentialCommandGroup(
                                new RetractOuttakeCommand(lift,arm,claw),
                                new InstantCommand(()->lift.setLiftPower(-0.2)),
                                new WaitCommand(300),
                                new InstantCommand(
                                        ()-> lift.brake_power()
                                )
                        ))
//                        runAtTime(5600, new InstantCommand(claw::open)), // extendo to first stack
//                        runAtTime(6900, new InstantCommand(claw::open)), // intake stack command
                )

////                        new ParallelCommandGroup(
////                                new ParallelCommandGroup(
////                                        new InstantCommand(extendo::mid),
////                                        new SequentialCommandGroup(
////                                                new WaitCommand(1800),
////                                                new TrajectorySequenceCommand(drive, untitled1) // approach board and then to other side of field
////                                        ),
////                                        new SequentialCommandGroup(
////                                                new WaitCommand(4500),
////                                                new InstantCommand(claw::open),
////                                                new IntakeStackCommand(hardwareMap,claw,intake, Intake.IntakePowers.FAST, extendo),
////                                                new SequentialCommandGroup(
////                                                        new WaitCommand(500),
////                                                        new InstantCommand(extendo::alpha)
////                                                )
////                                        )
////
////                                )
////                        ),
////                        new TrajectorySequenceCommand(drive, untitled2),
////                        new ParallelCommandGroup(
////                                new SequentialCommandGroup(
////                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
////                                        new InstantCommand(claw::open),
////                                        new WaitCommand(1000),
////                                        new RetractOuttakeCommand(lift,arm,claw),
////                                        new InstantCommand(()->lift.setLiftPower(-0.2)),
////                                        new WaitCommand(300),
////                                        new InstantCommand(
////                                                ()-> lift.brake_power()
////                                        )
////                                ),
////                                new ParallelCommandGroup(
////                                        new InstantCommand(extendo::mid),
////                                        new SequentialCommandGroup(
////                                                new WaitCommand(1500),
////                                                new TrajectorySequenceCommand(drive, untitled1) // approach board and then to other side of field
////                                        ),
////                                        new SequentialCommandGroup(
////                                                new WaitCommand(4500),
////                                                new InstantCommand(claw::open),
////                                                new IntakeStackCommand(hardwareMap,claw,intake, Intake.IntakePowers.FAST, extendo),
////                                                new SequentialCommandGroup(
////                                                        new WaitCommand(500),
////                                                        new InstantCommand(extendo::down)
////                                                )
////                                        )
////
////                                )
////                        ),
//                        new TrajectorySequenceCommand(drive, untitled3)
//                        new SequentialCommandGroup(
//                                new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
//                                new InstantCommand(claw::open),
//                                new WaitCommand(1000),
//                                new RetractOuttakeCommand(lift,arm,claw),
//                                new InstantCommand(()->lift.setLiftPower(-0.2)),
//                                new WaitCommand(300),
//                                new InstantCommand(
//                                        ()-> lift.brake_power()
//                                )
//                        )
//                );
        );
    };

    //                new SequentialCommandGroup(
//                        new TrajectorySequenceCommand(drive, ONE_GLOBAL),
//                        new ParallelCommandGroup(
//                                new TrajectorySequenceCommand(drive, THREE_PIXEL_ON_BACKDROP),
//                                new SequentialCommandGroup(
//                                    new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
//                                        new InstantCommand(claw::open),
//                                        new InstantCommand(()->lift.setLiftPower(-0.2)),
//                                        new WaitCommand(300),
//                                        new InstantCommand(()->lift.brake_power())
//                                )
//                        ),
//                        new ParallelCommandGroup(
//                                new TrajectorySequenceCommand(drive,FOUR_TO_LIGHTSPEED_BRIDGE_POSITION),
//                                new SequentialCommandGroup(
//                                    new RetractOuttakeCommand(lift,arm,claw),
//            //                        new WaitCommand(1000),
//                                    new InstantCommand(()->lift.setLiftPower(-0.2)),
//                                    new WaitCommand(700),
//                                    new InstantCommand(()->lift.brake_power())
//                                        )
//                        ),
//                        new ParallelCommandGroup(
//                                new ParallelRaceGroup(
//                                        new IntakeStackCommand(hardwareMap,claw,intake, Intake.IntakePowers.FAST, extendo),
//                                        new WaitCommand(6000)
//                                ),
//
//                                new InstantCommand(extendo::mid),
//                                new TrajectorySequenceCommand(drive, FIVE_INTAKE_PIXELS_STACK),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(500),
//                                        new InstantCommand(extendo::alpha)
//                                )
//                        ),
//                        new InstantCommand(extendo :: up),
//                        new TrajectorySequenceCommand(drive, SIX_LIGHTSPEED_BRIDGE_BACK),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
    //                                    new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.MID),
        //                                new InstantCommand(claw::open),
                //                        new InstantCommand(()->lift.setLiftPower(-0.2)),
                //                        new WaitCommand(700),
                //                        new InstantCommand(()->lift.brake_power())
//                                 ),
//                                new TrajectorySequenceCommand(drive, SEVEN_PIXEL_ON_BACKBOARD)
//                        ),
//                        new RetractOuttakeCommand(lift,arm,claw),
//                        new TrajectorySequenceCommand(drive, EIGHT_PARK_END)
//                )
}