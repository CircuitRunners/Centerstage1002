//package org.firstinspires.ftc.teamcode.auto.audience;
//
//import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.DEFAULT_PROP_LOCATION;
//import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.rotationConstant;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.ParallelRaceGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
//import org.firstinspires.ftc.teamcode.commands.IntakeStackCommand;
//import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
//import org.firstinspires.ftc.teamcode.commands.presets.MoveToScoringCommand;
//import org.firstinspires.ftc.teamcode.commands.presets.RetractOuttakeCommand;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.Arm;
//import org.firstinspires.ftc.teamcode.subsystems.Claw;
//import org.firstinspires.ftc.teamcode.subsystems.ExtendoArm;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Lift;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.utilities.PropLocation;
//import org.firstinspires.ftc.teamcode.utilities.Team;
//import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;
//
//// Enable photon if everything is already consistent.
////@Photon
//@Autonomous (name="RED AUDIENCE")
//public class RedAudience extends CommandOpMode {
//    // global trajectory definitions
//    private TrajectorySequence THREE_PIXEL_ON_BACKDROP;
//    private TrajectorySequence ONE_GLOBAL;
//
//    // Default Values
//    private PropLocation locationID = DEFAULT_PROP_LOCATION; // set to center by default
//
//    private Pose2d startPose = Pose2dMapped(-39.75, -63, Math.toRadians(90.00));
//
//    // Subsystems initialization
//    private Lift lift;
//    private Arm arm;
//    private Claw claw;
//    private ExtendoArm extendo;
//    private Intake intake;
//    private TeamPropDetector detector;
//
//    // Roadrunner Drive class
//    private SampleMecanumDrive drive;
//
//    @Override
//    public void initialize(){
//        // Schedule Bulk Cache Command
//        schedule(new BulkCacheCommand(hardwareMap));
//
//        // Initialize Mechanisms
//        detector = new TeamPropDetector(hardwareMap, true, Team.RED);
//        intake = new Intake(hardwareMap);
//        drive = new SampleMecanumDrive(hardwareMap);
//        lift = new Lift(hardwareMap);
//        claw = new Claw(hardwareMap);
//        arm = new Arm(hardwareMap);
//        extendo = new ExtendoArm(hardwareMap);
//
//        // Initialization for mechanism subsystems
//        lift.initialInitHang();
//        drive.setPoseEstimate(startPose);
//        detector.startStream();
//
//
//        while(opModeInInit()){
//            // The location id returns either LEFT, MIDDLE, or RIGHT.
//            // You will have to run location.getLocation() to retrieve the string of LEFT, etc.
//            locationID = detector.update();
//
//            telemetry.addLine("Ready for start!");
//            telemetry.addData("Prop", locationID.getLocation());
//
//            telemetry.update();
//        }
//
//        // Cleanup for other code
//        detector.stopStream();
//
//
//        switch (locationID) {
//            case LEFT: {
//                ONE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
//                        .lineToLinearHeading(Pose2dMapped(-43.71, -35.39, Math.toRadians(150.00)))
//                        .lineTo(Vector2dMapped(-36.20, -38.09))
//                        .lineToLinearHeading(Pose2dMapped(-34.32, -9.56, Math.toRadians(0.00)))
//                        .build();
//                THREE_PIXEL_ON_BACKDROP = drive.trajectorySequenceBuilder(Pose2dMapped(30.64, -11.30, Math.toRadians(0)))
//                        .lineTo(Vector2dMapped(32.64, -28.47))
//                        .lineTo(Vector2dMapped(48.72, -28.47))
//                        .build();
//                break;
//            }
//            case MIDDLE: {
//                ONE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
//                        .lineToLinearHeading(Pose2dMapped(-36.74, -33.91, Math.toRadians(90.00)))
//                        .lineTo(Vector2dMapped(-36.74, -36.66))
//                        .lineTo(Vector2dMapped(-58.54, -36.66))
//                        .lineTo(Vector2dMapped(-57.60, -25.57))
//                        .lineToLinearHeading(Pose2dMapped(-34.32, -9.56, Math.toRadians(0.00)))
//                        .build();
//
//                THREE_PIXEL_ON_BACKDROP = drive.trajectorySequenceBuilder(Pose2dMapped(30.64, -11.30, Math.toRadians(0)))
//                        .lineTo(Vector2dMapped(32.67, -36))
//                        .lineTo(Vector2dMapped(48.22, -36))
//                        .build();
//                break;
//            }
//            case RIGHT: {
//                ONE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
//                        .lineToLinearHeading(Pose2dMapped(-39.75, -35.13, Math.toRadians(30.00)))
//                        .lineTo(Vector2dMapped(-36.64, -35.13))
//                        .lineTo(Vector2dMapped(-42.53, -38.49))
//                        .lineToLinearHeading(Pose2dMapped(-34.32, -9.56, Math.toRadians(0.00)))
//                        .build();
//
//
//
//                THREE_PIXEL_ON_BACKDROP = drive.trajectorySequenceBuilder(Pose2dMapped(30.64, -11.30, Math.toRadians(0)))
//                        .lineTo(Vector2dMapped(32.64, -44))
//                        .lineTo(Vector2dMapped(52.22, -44))
//                        .build();
//                break;
//            }
//        }
//
//        TrajectorySequence FOUR_TO_LIGHTSPEED_BRIDGE_POSITION = drive.trajectorySequenceBuilder(ONE_GLOBAL.end())
//                .lineTo(Vector2dMapped(-55.5, -12.98))
//                .build();
//        // pickup from stack (move in)
//        TrajectorySequence FIVE_INTAKE_PIXELS_STACK = drive.trajectorySequenceBuilder(FOUR_TO_LIGHTSPEED_BRIDGE_POSITION.end())
//                .lineTo(Vector2dMapped(-60.22, -14.98))
//                .build();
//
//        TrajectorySequence BACK_TO_PIXEL_BACKBOARD = drive.trajectorySequenceBuilder(FIVE_INTAKE_PIXELS_STACK.end())
//                .lineTo(Vector2dMapped(32.64, -11.30))
//                .build();
//
//        TrajectorySequence INSERT_BACKBOARD = drive.trajectorySequenceBuilder(BACK_TO_PIXEL_BACKBOARD.end())
//                .lineTo(Vector2dMapped(51.27, -35.93))
//                .build();
//
//        TrajectorySequence STRAFE_PARK = drive.trajectorySequenceBuilder(INSERT_BACKBOARD.end())
//                .lineTo(Vector2dMapped(47.06, -37))
//                .lineTo(Vector2dMapped(47.06, -12.65))
//                .build();
//
//        // Theirs
//        schedule(
//                new SequentialCommandGroup(
//                        new TrajectorySequenceCommand(drive, ONE_GLOBAL),
//                        new TrajectorySequenceCommand(drive, FOUR_TO_LIGHTSPEED_BRIDGE_POSITION),
//                        new InstantCommand(claw::open),
//                        new ParallelCommandGroup(
//                            new ParallelRaceGroup(
//                                    new IntakeStackCommand(hardwareMap,claw,intake, Intake.IntakePowers.FAST, extendo),
//                                    new WaitCommand(6000)
//                            ),
//                            new InstantCommand(extendo::mid),
//                            new TrajectorySequenceCommand(drive, FIVE_INTAKE_PIXELS_STACK),
//                            new SequentialCommandGroup(
//                                    new WaitCommand(500),
//                                    new InstantCommand(extendo::alpha)
//                            )
//                        ),
//                        new InstantCommand(extendo :: up),
//                        new TrajectorySequenceCommand(drive,  BACK_TO_PIXEL_BACKBOARD),
//                        new ParallelCommandGroup(
//                            new SequentialCommandGroup(
//                                new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.MID),
//                                new InstantCommand(claw::open),
//                                new InstantCommand(()->lift.setLiftPower(-0.2)),
//                                new WaitCommand(300),
//                                new InstantCommand(()->lift.brake_power())
//                            )
//                        ),
//                        new TrajectorySequenceCommand(drive, THREE_PIXEL_ON_BACKDROP),
//                        new WaitCommand(300),
//                        new RetractOuttakeCommand(lift,arm,claw),
//                        new TrajectorySequenceCommand(drive, STRAFE_PARK)
//
//
//
//////                        new SequentialCommandGroup(
//////                            new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
//////                            new InstantCommand(claw::open)
//////                        ),
//////                        new ParallelCommandGroup(
//////                                new TrajectorySequenceCommand(drive,FOUR_TO_LIGHTSPEED_BRIDGE_POSITION_AND_INTAKE),
//////                                new SequentialCommandGroup(
//////                                    new RetractOuttakeCommand(lift,arm,claw),
//////                                    new InstantCommand(()->lift.setLiftPower(-0.2)),
//////                                    new WaitCommand(700),
//////                                    new InstantCommand(()->lift.brake_power())
//////                                )
//////                        ),
//////                        new ParallelCommandGroup(
//////                                new ParallelRaceGroup(
//////                                        new IntakeStackCommand(hardwareMap,claw,intake, Intake.IntakePowers.FAST, extendo),
//////                                        new WaitCommand(5000)
//////                                ),
//////                                new InstantCommand(extendo::mid),
//////                                new TrajectorySequenceCommand(drive, FIVE_INTAKE_PIXELS_STACK),
//////                                new SequentialCommandGroup(
//////                                        new WaitCommand(500),
//////                                        new InstantCommand(extendo::alpha)
//////                                )
//////                        ),
//////                        new TrajectorySequenceCommand(drive, BACK_TO_PIXEL_BACKBOARD),
//////                        new ParallelCommandGroup(
//////                                new SequentialCommandGroup(
//////                                    new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.MID),
//////                                    new InstantCommand(claw::open)
//////                                ),
//////                                new TrajectorySequenceCommand(drive, INSERT_BACKBOARD)
//////                        ),
//////                        new RetractOuttakeCommand(lift,arm,claw),
//////                        new TrajectorySequenceCommand(drive, STRAFE_PARK)
//               )
//        );
//    };
//
//    private double HeadingMapped (double heading) {
//        double mapper = rotationConstant;
//        return heading + mapper;
//    }
//
//    private double HMapRadians (double headingDeg) {
//        return HeadingMapped(Math.toRadians(headingDeg));
//    }
//    private double MathtoRadians (double toRad) {
//        return HMapRadians(toRad);
//    }
//
//    // Wrapper class for bogus mogus
//    public Vector2d Vector2dMapped(double x, double y) {
//        Vector2d conversionVector = new Vector2d(x,y);
//        conversionVector = conversionVector.rotated(rotationConstant);
//        Vector2d returnVector = new Vector2d(conversionVector.getX(), conversionVector.getY());
//        return returnVector;
//    }
//    public Pose2d Pose2dMapped(double x, double y, double heading) {
//        Vector2d conversionVector = new Vector2d(x,y);
//        conversionVector = conversionVector.rotated(rotationConstant);
//        return new Pose2d(conversionVector.getX(), conversionVector.getY(), HeadingMapped(heading));
//    }
//
//    public Pose2d Pose2dMapped(Pose2d pose) {
//        double x = pose.getX();
//        double y = pose.getY();
//        double heading = pose.getHeading();
//        Vector2d conversionVector = new Vector2d(x,y);
//        conversionVector = conversionVector.rotated(rotationConstant);
//        return new Pose2d(conversionVector.getX(), conversionVector.getY(), HeadingMapped(heading));
//    }
//
//}