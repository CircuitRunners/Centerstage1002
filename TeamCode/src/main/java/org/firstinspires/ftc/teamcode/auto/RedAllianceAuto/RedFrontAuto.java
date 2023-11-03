package org.firstinspires.ftc.teamcode.auto.RedAllianceAuto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Math.toRadians;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.DynamicConstants;
import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name="Red Front Auto")
public class RedFrontAuto {
    private double powerFullMultiplier = DynamicConstants.multiplier;
    private SampleMecanumDrive drive;

//    private BeaconDetector beaconDetector;
//    private BeaconDetector.BeaconTags beaconId = BeaconDetector.BeaconTags.LEFT;

    private Pose2d startPose = new Pose2d(0, 0, toRadians(0.0));
    private static double tile = 24;
    private static double half_tile = 12;
    private static double robot_len = 9;
    private static double square_edge = 1.5;

    private static Pose2d right= new Pose2d(12, -61.5, Math.toRadians(90));
    private static Pose2d left= new Pose2d(-36, -61.5, Math.toRadians(90));

    private DcMotorEx intake;
    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));


        intake = hardwareMap.get(DcMotorEx.class, "intake");
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        //RedFront
        TrajectorySequence centerLine = drive.trajectorySequenceBuilder(right)
                .forward(1.25 * tile - square_edge)
                .back(2)
                .strafeLeft(0.5 * tile)
                .forward(tile)
                .strafeRight(tile * 4.5 - square_edge)
                .build();

        TrajectorySequence rightLine = drive.trajectorySequenceBuilder(right)
                .splineToLinearHeading(new Pose2d(-34, -34), Math.toRadians(90))
                .back(2)
                .strafeLeft(tile)
                .forward(tile * 4 - square_edge)
                .build();

        TrajectorySequence leftLine = drive.trajectorySequenceBuilder(left)
                .splineTo(new Vector2d(-32, -35), Math.toRadians(0))
                .back(5)
                .strafeLeft(tile)
                .forward(tile * 4 - square_edge * 2)
                .build();


    }
    }
