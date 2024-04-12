package org.firstinspires.ftc.teamcode.procedures.auto._REMOVE_OLD.audience;

import static org.firstinspires.ftc.teamcode.controllers.common.utilities.CrossBindings.DEFAULT_PROP_LOCATION;
import static org.firstinspires.ftc.teamcode.controllers.common.utilities.CrossBindings.rotationConstant;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.controllers.auto.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Arm;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Claw;
import org.firstinspires.ftc.teamcode.controllers.subsytems.ExtendoArm;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Intake;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Lift;
import org.firstinspires.ftc.teamcode.controllers.auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.PropLocation;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.Team;
import org.firstinspires.ftc.teamcode.controllers.vision.TeamPropDetector;

// Enable photon if everything is already consistent.
//@Photon
@Autonomous (name="RICHARD AUDIENCE")
public class RichardAudience extends CommandOpMode {
    // global trajectory definitions
    private TrajectorySequence THREE_PIXEL_ON_BACKDROP;
    private TrajectorySequence ONE_GLOBAL;

    // Default Values
    private PropLocation locationID = DEFAULT_PROP_LOCATION; // set to center by default

    private Pose2d startPose = Pose2dMapped(-36.07, 62.63, Math.toRadians(270.00));

    // Subsystems initialization
    private Lift lift;
    private Arm arm;
    private Claw claw;
    private ExtendoArm extendo;
    private Intake intake;
    private TeamPropDetector detector;

    // Roadrunner Drive class
    private SampleMecanumDrive drive;

    @Override
    public void initialize(){
        // Schedule Bulk Cache Command
        schedule(new BulkCacheCommand(hardwareMap));

        // Initialize Mechanisms
        detector = new TeamPropDetector(hardwareMap, true, Team.BLUE);
        intake = new Intake(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        extendo = new ExtendoArm(hardwareMap);

        // Initialization for mechanism subsystems
        lift.initialInitHang();
        drive.setPoseEstimate(startPose);
        detector.startStream();


        while(opModeInInit()){
            // The location id returns either LEFT, MIDDLE, or RIGHT.
            // You will have to run location.getLocation() to retrieve the string of LEFT, etc.
            locationID = detector.update();

            telemetry.addLine("Ready for start!");
            telemetry.addData("Prop", locationID.getLocation());

            telemetry.update();
        }

        // Cleanup for other code
        detector.stopStream();


        switch (locationID) {
            case LEFT: {
                ONE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(Vector2dMapped(-36.46, 33.59))
                        .lineTo(Vector2dMapped(-37.24, 43.36))
                        .splineToLinearHeading(Pose2dMapped(-62.24, 36.85, Math.toRadians(360.00)), Math.toRadians(184.74))
                        .splineToConstantHeading(Vector2dMapped(-30.47, 59.37), Math.toRadians(32.61))
                        .splineToConstantHeading(Vector2dMapped(6.51, 58.59), Math.toRadians(-17.41))
                        .splineToConstantHeading(Vector2dMapped(37.37, 37.11), Math.toRadians(-0.35))
                        .build();

                break;
            }
            case MIDDLE: {
                ONE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(Vector2dMapped(-36.46, 33.59))
                        .lineTo(Vector2dMapped(-37.24, 43.36))
                        .splineToLinearHeading(Pose2dMapped(-62.24, 36.85, Math.toRadians(360.00)), Math.toRadians(184.74))
                        .splineToConstantHeading(Vector2dMapped(-30.47, 59.37), Math.toRadians(32.61))
                        .splineToConstantHeading(Vector2dMapped(6.51, 58.59), Math.toRadians(-17.41))
                        .splineToConstantHeading(Vector2dMapped(37.37, 37.11), Math.toRadians(-0.35))
                        .build();
                break;
            }
            case RIGHT: {
                ONE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(Vector2dMapped(-36.46, 33.59))
                        .lineTo(Vector2dMapped(-37.24, 43.36))
                        .splineToLinearHeading(Pose2dMapped(-62.24, 36.85, Math.toRadians(360.00)), Math.toRadians(184.74))
                        .splineToConstantHeading(Vector2dMapped(-30.47, 59.37), Math.toRadians(32.61))
                        .splineToConstantHeading(Vector2dMapped(6.51, 58.59), Math.toRadians(-17.41))
                        .splineToConstantHeading(Vector2dMapped(37.37, 37.11), Math.toRadians(-0.35))
                        .build();


                break;
            }
        }

        // Theirs
        schedule(
                new SequentialCommandGroup(
                       new TrajectorySequenceCommand(drive, ONE_GLOBAL)


////                        new SequentialCommandGroup(
////                            new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
////                            new InstantCommand(claw::open)
////                        ),
////                        new ParallelCommandGroup(
////                                new TrajectorySequenceCommand(drive,FOUR_TO_LIGHTSPEED_BRIDGE_POSITION_AND_INTAKE),
////                                new SequentialCommandGroup(
////                                    new RetractOuttakeCommand(lift,arm,claw),
////                                    new InstantCommand(()->lift.setLiftPower(-0.2)),
////                                    new WaitCommand(700),
////                                    new InstantCommand(()->lift.brake_power())
////                                )
////                        ),
////                        new ParallelCommandGroup(
////                                new ParallelRaceGroup(
////                                        new IntakeStackCommand(hardwareMap,claw,intake, Intake.IntakePowers.FAST, extendo),
////                                        new WaitCommand(5000)
////                                ),
////                                new InstantCommand(extendo::mid),
////                                new TrajectorySequenceCommand(drive, FIVE_INTAKE_PIXELS_STACK),
////                                new SequentialCommandGroup(
////                                        new WaitCommand(500),
////                                        new InstantCommand(extendo::alpha)
////                                )
////                        ),
////                        new TrajectorySequenceCommand(drive, BACK_TO_PIXEL_BACKBOARD),
////                        new ParallelCommandGroup(
////                                new SequentialCommandGroup(
////                                    new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.MID),
////                                    new InstantCommand(claw::open)
////                                ),
////                                new TrajectorySequenceCommand(drive, INSERT_BACKBOARD)
////                        ),
////                        new RetractOuttakeCommand(lift,arm,claw),
////                        new TrajectorySequenceCommand(drive, STRAFE_PARK)
                )
        );

    };

    @Override
    public void run(){
        drive.update();
        CommandScheduler.getInstance().run();

        telemetry.addData("lift", lift.getLiftPosition());
        telemetry.update();
    }

    private double HeadingMapped (double heading) {
        double mapper = rotationConstant;
        return heading + mapper;
    }

    private double HMapRadians (double headingDeg) {
        return HeadingMapped(Math.toRadians(headingDeg));
    }
    private double MathtoRadians (double toRad) {
        return HMapRadians(toRad);
    }

    // Wrapper class for bogus mogus
    public Vector2d Vector2dMapped(double x, double y) {
        Vector2d conversionVector = new Vector2d(x,y);
        conversionVector = conversionVector.rotated(rotationConstant);
        Vector2d returnVector = new Vector2d(conversionVector.getX(), conversionVector.getY());
        return returnVector;
    }
    public Pose2d Pose2dMapped(double x, double y, double heading) {
        Vector2d conversionVector = new Vector2d(x,y);
        conversionVector = conversionVector.rotated(rotationConstant);
        return new Pose2d(conversionVector.getX(), conversionVector.getY(), HeadingMapped(heading));
    }

    public Pose2d Pose2dMapped(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();
        Vector2d conversionVector = new Vector2d(x,y);
        conversionVector = conversionVector.rotated(rotationConstant);
        return new Pose2d(conversionVector.getX(), conversionVector.getY(), HeadingMapped(heading));
    }

}