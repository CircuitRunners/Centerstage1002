package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectorySequences {

    private double powerFullMultiplier = DynamicConstants.multiplier;
    private static SampleMecanumDrive drive;

//    private BeaconDetector beaconDetector;
//    private BeaconDetector.BeaconTags beaconId = BeaconDetector.BeaconTags.LEFT;

    private Pose2d startPose = new Pose2d(0, 0, toRadians(0.0));
    private static double tile = 24;
    private static double half_tile = 12;
    private static double robot_len = 9;
    private static double square_edge = 1.5;

    private static Pose2d back_red = new Pose2d(12, -61.5, Math.toRadians(90));
    private static Pose2d front_red = new Pose2d(-36, -61.5, Math.toRadians(90));
    private static Pose2d back_blue = new Pose2d(12, 61.5, Math.toRadians(-90));
    private static Pose2d front_blue = new Pose2d(-36, 61.5, Math.toRadians(-90));

    private DcMotorEx intake;



    //RedBack
    public static TrajectorySequence redBackCenterLine = drive.trajectorySequenceBuilder(back_red)
            .forward(1.25 * tile - square_edge)
            .back(1.1 * tile)
            .turn(Math.toRadians(-90))
            .build();

    public static TrajectorySequence redBackRightLine = drive.trajectorySequenceBuilder(back_red)
            .splineTo(new Vector2d(14, -35), Math.toRadians(0))
            .back(4)
            .strafeRight(tile)
            .forward(2)
            .build();

    public static TrajectorySequence redBackLeftLine = drive.trajectorySequenceBuilder(back_red)
            .splineTo(new Vector2d(9, -35), Math.toRadians(180))
            .back(4)
            .strafeLeft(tile)
            .turn(Math.toRadians(-180))
            .back(2)
            .build();



    //RedFront
    public static TrajectorySequence redFrontCenterLine = drive.trajectorySequenceBuilder(front_red)
            .forward(1.25 * tile - square_edge)
            .back(2)
            .strafeLeft(0.5 * tile)
            .forward(tile)
            .turn(Math.toRadians(-90))
            .forward(0.5*tile)
            .build();

    public static TrajectorySequence redFrontRightLine = drive.trajectorySequenceBuilder(front_red)
            .splineTo(new Vector2d(-32, -35), Math.toRadians(0))
            .back(5)
            .strafeLeft(tile)
            .build();

    public static TrajectorySequence redFrontLeftLine = drive.trajectorySequenceBuilder(front_red)
            .splineTo(new Vector2d(-38, -35), Math.toRadians(180))
            .back(5)
            .strafeRight(tile)
            .build();



    //BlueBack
    public static TrajectorySequence blueBackCenterLine = drive.trajectorySequenceBuilder(back_blue)
            .forward(1.25 * tile - square_edge)
            .back(1.1 * tile)
            .turn(Math.toRadians(90))
            .build();

    public static TrajectorySequence blueBackRightLine = drive.trajectorySequenceBuilder(back_blue)
            .splineTo(new Vector2d(9, 35), Math.toRadians(180))
            .back(4)
            .strafeRight(tile)
            .turn(Math.toRadians(180))
            .back(2)
            .build();

    public static TrajectorySequence blueBackLeftLine = drive.trajectorySequenceBuilder(back_blue)
            .splineTo(new Vector2d(14, 35), Math.toRadians(0))
            .back(4)
            .strafeLeft(tile)
            .forward(2)
            .build();




    //BlueFront
    public static TrajectorySequence blueFrontCenterLine = drive.trajectorySequenceBuilder(front_blue)
            .forward(1.25 * tile - square_edge)
            .back(2)
            .strafeRight(0.5 * tile)
            .forward(tile)
            .turn(Math.toRadians(90))
            .forward(0.5*tile)
            .build();


    public static TrajectorySequence blueFrontRightLine = drive.trajectorySequenceBuilder(front_blue)
            .splineTo(new Vector2d(-38, 34), Math.toRadians(180))
            .back(2)
            .strafeLeft(tile)
            .build();

    public static TrajectorySequence blueFrontLeftLine = drive.trajectorySequenceBuilder(front_blue)
            .splineTo(new Vector2d(-32, 35), Math.toRadians(0))
            .back(5)
            .strafeRight(tile)
            .build();




    //Parking
    public static TrajectorySequence parkFromBack = drive.trajectorySequenceBuilder(back_red)
            .forward(tile * 1.6 - square_edge)
            .build();

    public static TrajectorySequence parkFromFront = drive.trajectorySequenceBuilder(front_red)
            .forward(tile * 3.5 - square_edge)
            .build();



    //Yellow Pixel
    public static TrajectorySequence rightYellowPixel = drive.trajectorySequenceBuilder(back_red)
            .strafeLeft(0.5*tile)
            .build();
    public static TrajectorySequence leftYellowPixel = drive.trajectorySequenceBuilder(front_red)
            .strafeRight(0.5*tile)
            .build();

}
