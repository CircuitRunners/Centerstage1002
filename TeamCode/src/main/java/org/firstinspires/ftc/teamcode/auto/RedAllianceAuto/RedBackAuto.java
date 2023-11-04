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


@Autonomous(name="Red Back Auto")
public class RedBackAuto {
    private double powerFullMultiplier = DynamicConstants.multiplier;
    private SampleMecanumDrive drive;

//    private BeaconDetector beaconDetector;
//    private BeaconDetector.BeaconTags beaconId = BeaconDetector.BeaconTags.LEFT;

    private Pose2d startPose = new Pose2d(0, 0, toRadians(0.0));
    private static double tile = 24;
    private static double half_tile = 12;
    private static double robot_len = 9;
    private static double square_edge = 1.5;

    private static Pose2d right_red = new Pose2d(12, -61.5, Math.toRadians(90));
    private static Pose2d left_red = new Pose2d(-36, -61.5, Math.toRadians(90));
    private static Pose2d right_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
    private static Pose2d left_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));

    private DcMotorEx intake;

    public void initialize() {



        intake = hardwareMap.get(DcMotorEx.class, "intake");
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);


        //RedBack
        TrajectorySequence redBackCenterLine = drive.trajectorySequenceBuilder(right_red)
                .forward(1.25 * tile - square_edge)
                .back(1.1 * tile)
                .strafeRight(tile * 2)
                .build();

        TrajectorySequence redBackrightLine = drive.trajectorySequenceBuilder(right_red)
                .splineTo(new Vector2d(14, -35), Math.toRadians(0))
                .back(4)
                .strafeRight(tile)
                .forward(tile * 2 - square_edge * 2)
                .build();

        TrajectorySequence redBackleftLine = drive.trajectorySequenceBuilder(right_red)
                .splineTo(new Vector2d(9, -35), Math.toRadians(180))
                .back(4)
                .strafeLeft(tile)
                .back(tile * 2)
                .build();

        //RedFront
        TrajectorySequence redFrontCenterLine = drive.trajectorySequenceBuilder(left_red)
                .forward(1.25 * tile - square_edge)
                .back(2)
                .strafeLeft(0.5 * tile)
                .forward(tile)
                .strafeRight(tile * 4.5 - square_edge)
                .build();

        TrajectorySequence redFrontRightLine = drive.trajectorySequenceBuilder(left_red)
                .splineTo(new Vector2d(-32, -35), Math.toRadians(0))
                .back(5)
                .strafeLeft(tile)
                .forward(tile * 4 - square_edge)
                .build();

        TrajectorySequence redFrontLeftLine = drive.trajectorySequenceBuilder(left_red)
                .splineTo(new Vector2d(-38, -35), Math.toRadians(180))
                .back(5)
                .strafeRight(tile)
                .back(tile * 4 - square_edge * 2)
                .build();

        //BlueBack
        TrajectorySequence blueBackCenterLine = drive.trajectorySequenceBuilder(left_blue)
                .forward(1.25 * tile - square_edge)
                .back(1.1 * tile)
                .strafeLeft(tile * 2)
                .build();

        TrajectorySequence blueBackrightLine = drive.trajectorySequenceBuilder(left_blue)
                .splineTo(new Vector2d(9, 35), Math.toRadians(180))
                .back(4)
                .strafeRight(tile)
                .back(tile * 2 - square_edge * 2)
                .build();

        TrajectorySequence blueBackleftLine = drive.trajectorySequenceBuilder(left_blue)
                .splineTo(new Vector2d(14, 35), Math.toRadians(0))
                .back(4)
                .strafeLeft(tile)
                .forward(tile * 2 - square_edge * 2)
                .build();

        //BlueFront
        TrajectorySequence blueFrontCenterLine = drive.trajectorySequenceBuilder(left_blue)
                .forward(1.25 * tile - square_edge)
                .back(2)
                .strafeRight(0.5 * tile)
                .forward(tile)
                .strafeLeft(tile * 4.5 - square_edge)
                .build();

        TrajectorySequence blueFrontrightLine = drive.trajectorySequenceBuilder(left_blue)
                .splineTo(new Vector2d(-38, 34), Math.toRadians(180))
                .back(2)
                .strafeLeft(tile)
                .back(tile * 4 - square_edge)
                .build();

        TrajectorySequence blueFrontleftLine = drive.trajectorySequenceBuilder(left_blue)
                .splineTo(new Vector2d(-32, 35), Math.toRadians(0))
                .back(5)
                .strafeRight(tile)
                .forward(tile * 4 - square_edge * 2)
                .build();

    }
    }
