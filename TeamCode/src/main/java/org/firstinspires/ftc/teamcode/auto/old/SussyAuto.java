package org.firstinspires.ftc.teamcode.auto.old;
import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.rotationConstant;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous (name = "Sussylue Audience")
public class SussyAuto extends CommandOpMode{

    private TrajectorySequence THREE_PIXEL_ON_BACKDROP;
    private TrajectorySequence PURPLE_GLOBAL;
    private TrajectorySequence TO_STACK;
    private TrajectorySequence TO_BACKDROP;
    private TrajectorySequence TO_STACK_FROM_BACKDROP;
    private TrajectorySequence RETREAT_FROM_BACKDROP;


    // Default Values
    private PropLocation locationID = PropLocation.MIDDLE; // set to center by default

    private Pose2d startPose = Pose2dMapped(-39.75, 63, Math.toRadians(270.00));

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
    public void initialize (){
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
        detector.startStream();

        while(opModeInInit()){
            // The location id returns either LEFT, MIDDLE, or RIGHT.
            // You will have to run location.getLocation() to retrieve the string of LEFT, etc.
            locationID = detector.update();

            telemetry.addLine("Ready for start!");
            telemetry.addData("Prop", locationID.getLocation());

            telemetry.update();
        }


        detector.stopStream();

//        switch(locationID) {
//            case LEFT: {
//                PURPLE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
//                        .splineTo(Vector2dMapped(-35.84, 36.00), Math.toRadians(0.00))
//                        .build();
//                TO_STACK = drive.trajectorySequenceBuilder(PURPLE_GLOBAL.end())
//                        .lineTo(Vector2dMapped(-57.00, 36.00))
//                        .build();
//                TO_BACKDROP = drive.trajectorySequenceBuilder(TO_STACK.end())
//                        .lineTo(Vector2dMapped(-35.53, 58))
//                        .lineTo(Vector2dMapped(47.00, 58))
//                        .lineTo(Vector2dMapped(47.00, 43.89))
//                        .build();
//                RETREAT_FROM_BACKDROP = drive.trajectorySequenceBuilder(TO_BACKDROP.end())
//                        .lineTo(Vector2dMapped(40, 58))
//                        .build();
//                TO_STACK_FROM_BACKDROP = drive.trajectorySequenceBuilder(RETREAT_FROM_BACKDROP.end())
//                        .lineTo(Vector2dMapped(47.00, 58))
//                        .lineTo(Vector2dMapped(-35.53, 58))
//                        .lineTo(Vector2dMapped(-57.00, 36.00))
//                        .build();
//
//                break;
//            }
//            case MIDDLE: {
//                PURPLE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
//                        .lineToLinearHeading(Pose2dMapped(-36.75, 33.91, Math.toRadians(270)))
//                        .lineTo(Vector2dMapped(-39.75, 36.66))
//                        .build();
//
//                TO_STACK = drive.trajectorySequenceBuilder(PURPLE_GLOBAL.end())
//                        .lineToLinearHeading(Pose2dMapped(-57, 36, Math.toRadians(360)))
//                        .build();
//
//                TO_BACKDROP = drive.trajectorySequenceBuilder(TO_STACK.end())
//                        .lineTo(Vector2dMapped(47, 36))
//                        .build();
//
//                RETREAT_FROM_BACKDROP = drive.trajectorySequenceBuilder(TO_BACKDROP.end())
//                        .lineTo(Vector2dMapped(40, 36))
//                        .build();
//
//                TO_STACK_FROM_BACKDROP = drive.trajectorySequenceBuilder(RETREAT_FROM_BACKDROP.end())
//                        .lineToLinearHeading(Pose2dMapped(-57, 36, Math.toRadians(360)))
//                        .build();
//
//                break;
//
//            }
//            case RIGHT: {
//                PURPLE_GLOBAL = drive.trajectorySequenceBuilder(startPose)
//                        .splineTo(Vector2dMapped(-37.65, 36.00), Math.toRadians(180.00))
//                        .build();
//                TO_STACK = drive.trajectorySequenceBuilder(PURPLE_GLOBAL.end())
//                        .lineTo(Vector2dMapped(-57.00, 36.00))
//                        .build();
//                TO_BACKDROP = drive.trajectorySequenceBuilder(TO_STACK.end())
//                        .lineTo(Vector2dMapped(-35.53, 58))
//                        .lineTo(Vector2dMapped(47.00, 58))
//                        .lineTo(Vector2dMapped(47.00, 43.89))
//                        .build();
//                RETREAT_FROM_BACKDROP = drive.trajectorySequenceBuilder(TO_BACKDROP.end())
//                        .lineTo(Vector2dMapped(40, 58))
//                        .build();
//                TO_STACK_FROM_BACKDROP = drive.trajectorySequenceBuilder(RETREAT_FROM_BACKDROP.end())
//                        .lineTo(Vector2dMapped(47.00, 58))
//                        .lineTo(Vector2dMapped(-35.53, 58))
//                        .lineTo(Vector2dMapped(-57.00, 36.00))
//                        .build();
//
//                break;
//            }
//        }
        TrajectorySequence STRAFE_PARK = drive.trajectorySequenceBuilder(RETREAT_FROM_BACKDROP.end())
                    .lineTo(Vector2dMapped(47, 59.5))
                    .build();



            schedule(

                    new SequentialCommandGroup(
                            new TrajectorySequenceCommand(drive, STRAFE_PARK)
//                            new TrajectorySequenceCommand(drive, PURPLE_GLOBAL),
//                            new InstantCommand(claw::open),
//
//
//                            new ParallelCommandGroup(
//                                    new ParallelRaceGroup(
//                                            new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST, extendo),
//                                            new WaitCommand(6000)
//                                    ),
//                                    new InstantCommand(extendo::mid),
//                                    new TrajectorySequenceCommand(drive, TO_STACK),
//                                    new SequentialCommandGroup(
//                                            new WaitCommand(500),
//                                            new InstantCommand(extendo::mid)
//                                    )
//
//                            ),
//                            new TrajectorySequenceCommand(drive, TO_BACKDROP),
//
//                            new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.MID),
//                            new InstantCommand(claw::open),
//                            new InstantCommand(()->lift.setLiftPower(-0.2)),
//                            new WaitCommand(300),
//                            new InstantCommand(()->lift.brake_power()),
//
//                            new TrajectorySequenceCommand(drive, RETREAT_FROM_BACKDROP),
//                            new RetractOuttakeCommand(lift, arm, claw),
//
//
//                            new ParallelCommandGroup(
//                                    new ParallelRaceGroup(
//                                            new IntakeStackCommand(hardwareMap, claw, intake, Intake.IntakePowers.FAST, extendo),
//                                            new WaitCommand(6000)
//                                    ),
//                                    new InstantCommand(extendo::mid),
//                                    new TrajectorySequenceCommand(drive, TO_STACK_FROM_BACKDROP),
//                                    new SequentialCommandGroup(
//                                            new WaitCommand(500),
//                                            new InstantCommand(extendo::mid)
//                                    )
//
//                            ),
//
//                            new TrajectorySequenceCommand(drive, TO_BACKDROP),
//
//                            new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.MID),
//                            new InstantCommand(claw::open),
//                            new InstantCommand(()->lift.setLiftPower(-0.2)),
//                            new WaitCommand(300),
//                            new InstantCommand(()->lift.brake_power()),
//
//                            new TrajectorySequenceCommand(drive, RETREAT_FROM_BACKDROP),
//                            new RetractOuttakeCommand(lift, arm, claw),
//                            new TrajectorySequenceCommand(drive, STRAFE_PARK)
//
                    )

            );

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
