package org.firstinspires.ftc.teamcode.procedures.auto._REMOVE_OLD.backstage;

import static org.firstinspires.ftc.teamcode.controllers.common.utilities.CrossBindings.rotationConstant;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.controllers.commands.intake.IntakeStackCommand;
import org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.controllers.commands.presets.MoveToScoringCommand;
import org.firstinspires.ftc.teamcode.controllers.commands.presets.RetractOuttakeCommand;
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

// Complete! :) [who needs I&R anyways?]
@Autonomous (name="BLUE BACKSTAGE")
public class BlueStage extends CommandOpMode {
    private SampleMecanumDrive drive;
    private TrajectorySequence ONE_GLOBAL;

    private TrajectorySequence THREE_PIXEL_ON_BACKDROP;

    private TeamPropDetector detector;
    private PropLocation locationID = PropLocation.MIDDLE; // set to center by default

    //    private Pose2d startPose = Pose2dMapped(9.00, -61.50, Math.toRadians(90.00));
    private Pose2d startPose = Pose2dMapped(13.80, 63, Math.toRadians(270.00));
    private Lift lift;
    private Arm arm;
    private Claw claw;
    private ExtendoArm extendo;

    private Intake intake;

    @Override
    public void initialize(){
        detector = new TeamPropDetector(hardwareMap, true, Team.BLUE);
        schedule(new BulkCacheCommand(hardwareMap));

        intake = new Intake(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        lift = new Lift(hardwareMap);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        extendo = new ExtendoArm(hardwareMap);

        lift.initialInitHang();

        drive.setPoseEstimate(startPose);

        TrajectorySequence testTopTrajectory = drive.trajectorySequenceBuilder(Pose2dMapped(-40, -62.5, Math.toRadians(270.00)))
                .splineTo(Vector2dMapped(-54.24, -39.03), Math.toRadians(270.00))
                .splineTo(Vector2dMapped(-42.66, -27.05), Math.toRadians(210.00))
                .lineToLinearHeading(Pose2dMapped(-56.25, -25.30, Math.toRadians(180.00)))
                .lineTo(Vector2dMapped(-56.12, -11.17))
                .lineTo(Vector2dMapped(39.57, -11.30))
                .lineTo(Vector2dMapped(51.68, -36.07))
                .lineToLinearHeading(Pose2dMapped(35.53, -11.44, Math.toRadians(0.00)))
                .lineTo(Vector2dMapped(-60.29, -11.04))
                .lineToLinearHeading(Pose2dMapped(43.33, -11.30, Math.toRadians(180.00)))
                .lineTo(Vector2dMapped(51.41, -35.93))
                .build();


        TrajectorySequence purplePixelBackboard = drive.trajectorySequenceBuilder(startPose)
                .lineTo(Vector2dMapped(19.5, -40))
                .splineToConstantHeading(Vector2dMapped(20, -44), Math.toRadians(270.00))
                .splineToConstantHeading(Vector2dMapped(33, -51), Math.toRadians(270.00))
                .waitSeconds(0.1)
                .splineToLinearHeading(Pose2dMapped(44.51, -36.70,Math.toRadians(180)), Math.toRadians(180))
                .build();

        TrajectorySequence depositBackboardAndWithdraw = drive.trajectorySequenceBuilder(purplePixelBackboard.end())
                .lineTo(Vector2dMapped(51, -39))
                .waitSeconds(1)
                .lineTo(Vector2dMapped(45.51, -39))
                .build();

        TrajectorySequence toStacks = drive.trajectorySequenceBuilder(depositBackboardAndWithdraw.end())
                .splineToConstantHeading(Vector2dMapped(11.71, -36.74), Math.toRadians(270.00))
                .lineTo(Vector2dMapped(-58, -40.34))

                .build();
        TrajectorySequence backupFromStacksForPixelIntake = drive.trajectorySequenceBuilder(toStacks.end())
                .lineTo(Vector2dMapped(-54.5, -40.34))
                .build();

        TrajectorySequence backToRightSide = drive.trajectorySequenceBuilder(backupFromStacksForPixelIntake.end())
                .lineTo(Vector2dMapped(52.25, -36.07))
                .build();

        detector.startStream();

        while(opModeInInit()){
            locationID = detector.update();
            //telemetry.addLine("Ready for start!");
            telemetry.addData("Prop", locationID.getLocation());
            telemetry.update();
        }

        detector.stopStream();

        // Purple pixel
        switch (locationID) {
            case LEFT: {
                ONE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(Vector2dMapped(26.63, 39.57))
                        .lineTo(Vector2dMapped(26.83, 42.57))
                        .lineTo(Vector2dMapped(32.30, 45.89))
                        .build();
                THREE_PIXEL_ON_BACKDROP = drive.trajectorySequenceBuilder(ONE_GLOBAL.end())
                        .splineToLinearHeading(Pose2dMapped(48.31, 45.47, Math.toRadians(360.00)), MathtoRadians(-2.5))
                        .lineTo(Vector2dMapped(52.72, 45.47))
                        .build();
                break;
            }
            case MIDDLE: {
                ONE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(Vector2dMapped(12.3800001, 34.24))
                        .lineTo(Vector2dMapped(32.30, 45.89))
                        .build();
                THREE_PIXEL_ON_BACKDROP = drive.trajectorySequenceBuilder(ONE_GLOBAL.end())
                        .splineToLinearHeading(Pose2dMapped(48.31, 37, Math.toRadians(360.00)), MathtoRadians(-2.5))
                        .lineTo(Vector2dMapped(52.72, 37))
                        .build();
                break;
            }
            case RIGHT: {
                ONE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(Pose2dMapped(18.65, 44.64, Math.toRadians(205.00)))
                        .lineTo(Vector2dMapped(9.95, 39))
                        .lineToLinearHeading(Pose2dMapped(38.4, 45.89, Math.toRadians(362.5)))
                        .build();
                THREE_PIXEL_ON_BACKDROP = drive.trajectorySequenceBuilder(ONE_GLOBAL.end())
                        .splineToLinearHeading(Pose2dMapped(48.31, 30.97, Math.toRadians(360.00)), MathtoRadians(-2.5))
                        .lineTo(Vector2dMapped(52.72, 30.97))
                        .build();
                break;
            }
        }

//        TrajectorySequence THREE_PIXEL_ON_BACKDROP = drive.trajectorySequenceBuilder(ONE_GLOBAL.end())
//                .splineToLinearHeading(Pose2dMapped(48.31, -36.47, Math.toRadians(0.00)), MathtoRadians(0.00))
//                .lineTo(Vector2dMapped(52.22, -36.47))
//                .build();

        // Stack
        TrajectorySequence FOUR_TO_LIGHTSPEED_BRIDGE_POSITION = drive.trajectorySequenceBuilder(THREE_PIXEL_ON_BACKDROP.end())
                .lineTo(Vector2dMapped(33.24, 12.25))
                .lineTo(Vector2dMapped(-52.22, 12.98))
                .build();

        // Back 2 The Backboard
        TrajectorySequence FIVE_INTAKE_PIXELS_STACK = drive.trajectorySequenceBuilder(FOUR_TO_LIGHTSPEED_BRIDGE_POSITION.end())
                .lineTo(Vector2dMapped(-54.29, 12.98))
                .build();

        TrajectorySequence SIX_LIGHTSPEED_BRIDGE_BACK = drive.trajectorySequenceBuilder(FIVE_INTAKE_PIXELS_STACK.end())
                .lineTo(Vector2dMapped(39.84, 12.11))
                .build();

        TrajectorySequence SEVEN_PIXEL_ON_BACKBOARD = drive.trajectorySequenceBuilder(SIX_LIGHTSPEED_BRIDGE_BACK.end())
                .lineTo(Vector2dMapped(42.80, 31.76))
                .lineTo(Vector2dMapped(53.85, 36.99))
                .build();

        TrajectorySequence EIGHT_PARK_END = drive.trajectorySequenceBuilder(SEVEN_PIXEL_ON_BACKBOARD.end())
                .lineTo(Vector2dMapped(48.72, 26.24))
                .lineTo(Vector2dMapped(51.54, 11.98))
                .lineTo(Vector2dMapped(64.06, 12.65))
                .build();

        schedule(
//                new SequentialCommandGroup(
//                        new TrajectorySequenceCommand(drive,testTopTrajectory)
//                )
                new SequentialCommandGroup(
                        new TrajectorySequenceCommand(drive, ONE_GLOBAL),
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, THREE_PIXEL_ON_BACKDROP),
                                new SequentialCommandGroup(
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
                                        new InstantCommand(claw::open),
                                        new InstantCommand(()->lift.setLiftPower(-0.2)),
                                        new WaitCommand(300),
                                        new InstantCommand(()->lift.brake_power())
                                )
                        ),
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive,FOUR_TO_LIGHTSPEED_BRIDGE_POSITION),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new RetractOuttakeCommand(lift,arm,claw),
                                        //                        new WaitCommand(1000),
                                        new InstantCommand(()->lift.setLiftPower(-0.2)),
                                        new WaitCommand(700),
                                        new InstantCommand(()->lift.brake_power())
                                )
                        ),
                        new ParallelCommandGroup(
                                new ParallelRaceGroup(
                                        new WaitCommand(6000),
                                        new IntakeStackCommand(hardwareMap,claw,intake, Intake.IntakePowers.FAST)
                                ),
                                new InstantCommand(extendo::mid),
                                new TrajectorySequenceCommand(drive, FIVE_INTAKE_PIXELS_STACK)
                        ),
                        new InstantCommand(extendo :: up),
                        new TrajectorySequenceCommand(drive, SIX_LIGHTSPEED_BRIDGE_BACK),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.MID),
                                        new InstantCommand(claw::open)
                                ),
                                new TrajectorySequenceCommand(drive, SEVEN_PIXEL_ON_BACKBOARD)
                        ),
                        new RetractOuttakeCommand(lift,arm,claw),
                        new TrajectorySequenceCommand(drive, EIGHT_PARK_END)
                )
        );
    };

//    @Override
//    public void run() {
//        super.run();
//        CommandScheduler.getInstance().run();
//        telemetry.update();
//    }


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
        com.acmerobotics.roadrunner.geometry.Vector2d conversionVector = new com.acmerobotics.roadrunner.geometry.Vector2d(x,y);
        conversionVector = conversionVector.rotated(rotationConstant);
        Vector2d returnVector = new Vector2d(conversionVector.getX(), conversionVector.getY());
        return returnVector;
    }
    public Pose2d Pose2dMapped(double x, double y, double heading) {
        com.acmerobotics.roadrunner.geometry.Vector2d conversionVector = new com.acmerobotics.roadrunner.geometry.Vector2d(x,y);
        conversionVector = conversionVector.rotated(rotationConstant);
        return new Pose2d(conversionVector.getX(), conversionVector.getY(), HeadingMapped(heading));
    }

    public Pose2d Pose2dMapped(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();
        com.acmerobotics.roadrunner.geometry.Vector2d conversionVector = new com.acmerobotics.roadrunner.geometry.Vector2d(x,y);
        conversionVector = conversionVector.rotated(rotationConstant);
        return new Pose2d(conversionVector.getX(), conversionVector.getY(), HeadingMapped(heading));
    }

}