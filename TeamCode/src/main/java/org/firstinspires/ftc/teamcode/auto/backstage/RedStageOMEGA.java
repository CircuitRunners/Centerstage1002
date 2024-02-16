package org.firstinspires.ftc.teamcode.auto.backstage;

import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.rotationConstant;
import static org.firstinspires.ftc.teamcode.utilities.PropLocation.RIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
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
    private TrajectorySequence ONE_GLOBAL;

    private TrajectorySequence THREE_PIXEL_ON_BACKDROP;

    private TeamPropDetector detector;

    private PropLocation locationID = PropLocation.MIDDLE; // set to center by default

//    private Pose2d startPose = Pose2dMapped(9.00, -61.50, Math.toRadians(90.00));
//    private Pose2d startPose = Pose2dMapped(10.25, -63,  Math.toRadians(90.00));
//    public Pose2d startPose = Pose2dMapped(10.5, -62.5,  Math.toRadians(90.00));

    double center_line_y = -12.5, stack_x = -55.5, avoidance_x_constant = 1;

    Pose2d startPose = new Pose2d(10.5, -62.5,  Math.toRadians(90.00));

    Pose2d pixel_left = new Pose2d(22.4, -42, Math.toRadians(90.00));
    Pose2d pixel_center = new Pose2d(11.84, -33.90, Math.toRadians(90.00));
    Pose2d pixel_right = new Pose2d(27, -40, Math.toRadians(105));


    double offsetFromBoard = 4.0;
    Pose2d boardPosition_left = new Pose2d(50.5, -32 - offsetFromBoard, Math.toRadians(0));
    Pose2d boardPosition_center = new Pose2d(50.5, -35 - offsetFromBoard, Math.toRadians(0.00));
    Pose2d boardPosition_right = new Pose2d(50.5, -38 - offsetFromBoard, Math.toRadians(0));

    Pose2d purplePixel = pixel_right;
    Pose2d boardPosition = boardPosition_center;

    double fastVelocity = 60;

    private Lift lift;
    private Arm arm;
    private Claw claw;
    private ExtendoArm extendo;

    private Intake intake;

    private Integer iteration = 0;

    @Override
    public void initialize(){
//        detector = new TeamPropDetector(hardwareMap, true, Team.RED);
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

        // Left Trajectories


        TrajectorySequence toPixelLeft = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(8, -37.5), Math.toRadians(135.00)) // left
                .setReversed(true)
                .splineToLinearHeading(boardPosition, Math.toRadians(0))
                .build();

        TrajectorySequence toPixelCenter = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(purplePixel) // center pixel
                .setReversed(true)
                .splineToLinearHeading(boardPosition, Math.toRadians(0))
                .build();

        TrajectorySequence toPixelRight = drive.trajectorySequenceBuilder(startPose)
               .splineToSplineHeading(purplePixel, Math.toRadians(45)) // right pixel
                .setReversed(true)
                .splineToLinearHeading(boardPosition, Math.toRadians(0))
                .build();

        TrajectorySequence toPixelStack = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                // move smoothly away from the board to the launch point to go across the field to stack
                .splineToConstantHeading(new Vector2d(23.55, center_line_y), Math.toRadians(180.00))
                //* set the speed to be greater to zoom faster
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, 14.28))
                // zoom across to the pixel stack
                .splineTo(new Vector2d(stack_x, center_line_y), Math.toRadians(180.00))
                .build();

        TrajectorySequence toBoardAgain = drive.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                // zoom across the middle of the field
                .splineTo(new Vector2d(23.55, center_line_y), Math.toRadians(0.00))
                .resetVelConstraint()
                // back to the board
                .splineToLinearHeading(boardPosition, Math.toRadians(0.00))
                .build();

        TrajectorySequence toBoardThenStack = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                // move smoothly away from the board to the launch point to go across the field to stack
                .splineToConstantHeading(new Vector2d(23.55, center_line_y), Math.toRadians(180.00))
                //* set the speed to be greater to zoom faster
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, 14.28))
                // zoom across to the pixel stack
                .splineTo(new Vector2d(stack_x - avoidance_x_constant, center_line_y), Math.toRadians(180.00))
                .strafeRight(12)
                .strafeLeft(12)
                .setReversed(false)
                .build();


//        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(Pose2dMapped(11.84, -33.90, Math.toRadians(90.00)))
//                .lineToLinearHeading(Pose2dMapped(23.28, -43.74, Math.toRadians(90.00)))
//                .splineToLinearHeading(Pose2dMapped(50.5, -37.75, Math.toRadians(0.0001)), MathtoRadians(0.0001))
//                .build();
//
//        TrajectorySequence untitled1 = drive.trajectorySequenceBuilder(untitled0.end())
//                .setReversed(true)
////                .splineTo(Vector2dMapped(28.93, -11.84), MathtoRadians(180.00))
//                .splineTo(Vector2dMapped(23.55, -13), MathtoRadians(180.00))
//                .splineTo(Vector2dMapped(-54.5, -13), MathtoRadians(180.00)) // -59.something
//                .build();
//
//        TrajectorySequence untitled2 = drive.trajectorySequenceBuilder(untitled1.end())
//                .setReversed(false)
////                .splineTo(Vector2dMapped(28.93, -11.84), MathtoRadians(180.00))
//                .splineTo(Vector2dMapped(23.55, -13), MathtoRadians(0))
//                .splineToLinearHeading(Pose2dMapped(51.25, -37.75, Math.toRadians(0.00)), MathtoRadians(0.00))
//                .build();
        TrajectorySequence untitled3 = drive.trajectorySequenceBuilder(startPose)
               .lineToLinearHeading(purplePixel) // center pixel
//               .splineToSplineHeading(purplePixel, Math.toRadians(45)) // right pixel
//                .splineTo(new Vector2d(8, -37.5), Math.toRadians(135.00))
                .setReversed(true)
                .splineToLinearHeading(boardPosition, Math.toRadians(0))

                .setReversed(true)
                // move smoothly away from the board to the launch point to go across the field to stack
                .splineToConstantHeading(new Vector2d(23.55, center_line_y), Math.toRadians(180.00))
                //* set the speed to be greater to zoom faster
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, 14.28))
                // zoom across to the pixel stack
                .splineTo(new Vector2d(stack_x, center_line_y), Math.toRadians(180.00))


                .setReversed(false)
                // zoom across the middle of the field
                .splineTo(new Vector2d(23.55, center_line_y), Math.toRadians(0.00))
                .resetVelConstraint()
                // back to the board
                .splineToLinearHeading(boardPosition, Math.toRadians(0.00))

                .setReversed(true)
                // move smoothly away from the board to the launch point to go across the field to stack
                .splineToConstantHeading(new Vector2d(23.55, center_line_y), Math.toRadians(180.00))
                //* set the speed to be greater to zoom faster
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, 14.28))
                // zoom across to the pixel stack
                .splineTo(new Vector2d(stack_x, center_line_y), Math.toRadians(180.00))


                .setReversed(false)
                // zoom across the middle of the field
                .splineTo(new Vector2d(23.55, center_line_y), Math.toRadians(0.00))
                .resetVelConstraint()
                // back to the board
                .splineToLinearHeading(boardPosition, Math.toRadians(0.00))

                .setReversed(true)
                // move smoothly away from the board to the launch point to go across the field to stack
                .splineToConstantHeading(new Vector2d(23.55, center_line_y), Math.toRadians(180.00))
                //* set the speed to be greater to zoom faster
                .setVelConstraint(new MecanumVelocityConstraint(fastVelocity, 14.28))
                // zoom across to the pixel stack
                .splineTo(new Vector2d(stack_x - avoidance_x_constant, center_line_y), Math.toRadians(180.00))
                .strafeRight(12)
                .strafeLeft(12)
                .setReversed(false)

                // zoom across the middle of the field
                .splineTo(new Vector2d(23.55, center_line_y), Math.toRadians(0.00))
                .resetVelConstraint()
                // back to the board
                .splineToLinearHeading(boardPosition, Math.toRadians(0.00))

//                                        // come back from the board a tiny bit
//                                       .lineToConstantHeading(new Vector2d(46.54, -27.71))
//                                       // move up to the park zone and turn so that dillans forward is close to forward
//                                       .lineToSplineHeading(new Pose2d(46.36, -12.82, Math.toRadians(90)))

                .build();





        detector.startStream();
//
        while(opModeInInit()){
            locationID = detector.update();
            telemetry.addLine("Ready for start!");
            telemetry.addData("Prop", locationID.getLocation());
            telemetry.update();
        }
//
        detector.stopStream();

        // Purple pixel
//        switch (locationID) {
//            case LEFT: {
//
//              break;
//            }
//            case MIDDLE: {
//
//                break;
//            }
//            case RIGHT: {
//
//                break;
//            }
//        }

        schedule(
                new SequentialCommandGroup(
                        new TrajectorySequenceCommand(drive, toPixelCenter),
                        new TrajectorySequenceCommand(drive, toBoardAgain),
                        new TrajectorySequenceCommand(drive, toPixelStack),
                        new TrajectorySequenceCommand(drive, toBoardAgain),
                        new TrajectorySequenceCommand(drive, toPixelStack),
                        new TrajectorySequenceCommand(drive, toBoardThenStack),
                        new TrajectorySequenceCommand(drive, toBoardAgain)
                )
//                new SequentialCommandGroup(
////                        new ParallelCommandGroup(
////                                new TrajectorySequenceCommand(drive, untitled0), // push pixel, come back, approach board spline
////                                new SequentialCommandGroup(
////                                        new WaitCommand(5000),
////                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
////                                        new InstantCommand(claw::open),
////                                        new WaitCommand(1500),
////                                        new RetractOuttakeCommand(lift,arm,claw),
////                                        new InstantCommand(()->lift.setLiftPower(-0.2)),
////                                        new WaitCommand(300),
////                                        new InstantCommand(
////                                                ()-> lift.brake_power()
////                                        )
////                                )
////                        ),
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

//    @Override
//    public void run() {
//        super.run();
//        CommandScheduler.getInstance().run();
//        telemetry.update();
//    }


}