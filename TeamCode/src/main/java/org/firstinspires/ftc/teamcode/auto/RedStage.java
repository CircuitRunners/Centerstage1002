package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.rotationConstant;
import static java.lang.Math.toRadians;

import android.icu.util.ULocale;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommandEx;
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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

// Complete! :) [who needs I&R anyways?]
@Autonomous (name="Parking Aut@@@@swo (Blue, Audience)")
public class RedStage extends CommandOpMode {

    private double powerFullMultiplier = DynamicConstants.multiplier;
    private SampleMecanumDrive drive;

    private TeamPropDetector detector;
    private int locationID = 1; // set to center by default

//    private Pose2d startPose = Pose2dMapped(9.00, -61.50, Math.toRadians(90.00));
    private Pose2d startPose =Pose2dMapped(10.50, -62.5, Math.toRadians(90.00));
    private static double tile = 24;
    private static double half_tile = 12;
    private static double robot_len = 9;
    private static double square_edge = 1.5;

    private static Pose2d right= new Pose2d(12, -61.5, Math.toRadians(90));
    private static Pose2d left= new Pose2d(-36, -61.5, Math.toRadians(90));

    private Lift lift;
    private Arm arm;
    private Claw claw;
    private ExtendoArm extendo;

    private Intake intake;

    public RedStage() {
    }

    @Override
    public void initialize(){

        //detector = new TeamPropDetector(hardwareMap, true);
        schedule(new BulkCacheCommand(hardwareMap));

        intake = new Intake(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        lift = new Lift(hardwareMap);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        extendo = new ExtendoArm(hardwareMap);

        TrajectorySequence testTopTrajectory = drive.trajectorySequenceBuilder(Pose2dMapped(10.50, -62.5, Math.toRadians(90.00)))
                .lineTo(Vector2dMapped(8, -33.00))
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(startPose)
                .lineTo(Vector2dMapped(19.5, -40))
                .splineToConstantHeading(Vector2dMapped(20, -44), Math.toRadians(90.00))
                .splineToConstantHeading(Vector2dMapped(33, -51), Math.toRadians(90.00))
                .waitSeconds(0.1)
                .splineToLinearHeading(Pose2dMapped(44.51, -36.70,Math.toRadians(0)), Math.toRadians(0.00))
                .build();

        TrajectorySequence wtfareweon = drive.trajectorySequenceBuilder(leftPark.end())
                .lineTo(Vector2dMapped(51, -39))
                .waitSeconds(1)
                .lineTo(Vector2dMapped(45.51, -39))
                .build();

        TrajectorySequence afterWtf = drive.trajectorySequenceBuilder(wtfareweon.end())
                .splineToConstantHeading(Vector2dMapped(11.71, -36.74), Math.toRadians(90.00))
                .lineTo(Vector2dMapped(-58, -36.34))

                .build();
        TrajectorySequence afterWTFBkp = drive.trajectorySequenceBuilder(afterWtf.end())
                .lineTo(Vector2dMapped(-54.5, -36.34))
                .build();

        TrajectorySequence after2W = drive.trajectorySequenceBuilder(afterWTFBkp.end())
                .lineTo(Vector2dMapped(52.22, -36.07))
                .build();


//        detector.startStream();
        while(opModeInInit()){
            //locationID = detector.update();
            //telemetry.addLine("Ready for start!");
            //telemetry.addData("Prop", locationID);
            telemetry.update();
        }
        drive.setPoseEstimate(testTopTrajectory.start());

//        detector.stopStream();

        schedule(
//                new SequentialCommandGroup(
//                        new TrajectorySequenceCommand(drive,testTopTrajectory)
//                )
                new SequentialCommandGroup(
                        new TrajectorySequenceCommand(drive, leftPark),
                        new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT),
                        new WaitCommand(500),
                        new InstantCommand(claw::open),
                        new TrajectorySequenceCommand(drive, wtfareweon),
                        new RetractOuttakeCommand(lift,arm,claw),
                        new WaitCommand(1000),
                        new InstantCommand(()->lift.setLiftPower(-0.1)),
                        new WaitCommand(1000),
                        new InstantCommand(()->lift.brake_power()),
                        new InstantCommand(extendo::up),
                        new TrajectorySequenceCommand(drive,afterWtf),
                        new ParallelCommandGroup(
                                new IntakeCommandEx(hardwareMap,claw,intake, Intake.IntakePowers.SLOW),
                                new InstantCommand(extendo::down),
                                new TrajectorySequenceCommand(drive, afterWTFBkp)
                        ),
                        new TrajectorySequenceCommand(drive, after2W)
                )
        );
    };

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