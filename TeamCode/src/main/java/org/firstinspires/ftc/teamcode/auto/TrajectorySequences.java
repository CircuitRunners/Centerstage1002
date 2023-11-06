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

    public static TrajectorySequence redBackCenterLine;
    public static TrajectorySequence redBackRightLine;
    public static TrajectorySequence redBackLeftLine;

    public static TrajectorySequence redFrontCenterLine;
    public static TrajectorySequence redFrontRightLine;
    public static TrajectorySequence redFrontLeftLine;

    public static TrajectorySequence blueBackCenterLine;
    public static TrajectorySequence blueBackRightLine;
    public static TrajectorySequence blueBackLeftLine;

    public static TrajectorySequence blueFrontCenterLine;
    public static TrajectorySequence blueFrontRightLine;
    public static TrajectorySequence blueFrontLeftLine;

    public static TrajectorySequence parkFromRedBack;
    public static TrajectorySequence parkFromRedFront;
    public static TrajectorySequence parkFromBlueBack;
    public static TrajectorySequence parkFromBlueFront;
    public static TrajectorySequence redFrontYellowPixel;
    public static TrajectorySequence redBackYellowPixel;
    public static TrajectorySequence blueFrontYellowPixel;
    public static TrajectorySequence blueBackYellowPixel;



static void generateTrajectories() {

    //RedBack
    redBackCenterLine = drive.trajectorySequenceBuilder(back_red)
            .forward(1.25 * tile - square_edge)
            .back(1.1 * tile)
            .turn(Math.toRadians(-90))
            .build();

    redBackRightLine = drive.trajectorySequenceBuilder(back_red)
            .splineTo(new Vector2d(14, -35), Math.toRadians(0))
            .back(4)
            .strafeRight(tile)
            .forward(2)
            .build();

    redBackLeftLine = drive.trajectorySequenceBuilder(back_red)
            .splineTo(new Vector2d(9, -35), Math.toRadians(180))
            .back(4)
            .strafeLeft(tile)
            .turn(Math.toRadians(-180))
            .back(2)
            .build();


    //RedFront
    redFrontCenterLine = drive.trajectorySequenceBuilder(front_red)
            .forward(1.25 * tile - square_edge)
            .back(2)
            .strafeLeft(0.5 * tile)
            .forward(tile)
            .turn(Math.toRadians(-90))
            .forward(0.5 * tile)
            .build();

    redFrontRightLine = drive.trajectorySequenceBuilder(front_red)
            .splineTo(new Vector2d(-32, -35), Math.toRadians(0))
            .back(5)
            .strafeLeft(tile)
            .build();

    redFrontLeftLine = drive.trajectorySequenceBuilder(front_red)
            .splineTo(new Vector2d(-38, -35), Math.toRadians(180))
            .back(5)
            .strafeRight(tile)
            .build();


    //BlueBack
    blueBackCenterLine = drive.trajectorySequenceBuilder(back_blue)
            .forward(1.25 * tile - square_edge)
            .back(1.1 * tile)
            .turn(Math.toRadians(90))
            .build();

    blueBackRightLine = drive.trajectorySequenceBuilder(back_blue)
            .splineTo(new Vector2d(9, 35), Math.toRadians(180))
            .back(4)
            .strafeRight(tile)
            .turn(Math.toRadians(180))
            .back(2)
            .build();

    blueBackLeftLine = drive.trajectorySequenceBuilder(back_blue)
            .splineTo(new Vector2d(14, 35), Math.toRadians(0))
            .back(4)
            .strafeLeft(tile)
            .forward(2)
            .build();


    //BlueFront
    blueFrontCenterLine = drive.trajectorySequenceBuilder(front_blue)
            .forward(1.25 * tile - square_edge)
            .back(2)
            .strafeRight(0.5 * tile)
            .forward(tile)
            .turn(Math.toRadians(90))
            .forward(0.5 * tile)
            .build();


    blueFrontRightLine = drive.trajectorySequenceBuilder(front_blue)
            .splineTo(new Vector2d(-38, 34), Math.toRadians(180))
            .back(2)
            .strafeLeft(tile)
            .build();

    blueFrontLeftLine = drive.trajectorySequenceBuilder(front_blue)
            .splineTo(new Vector2d(-32, 35), Math.toRadians(0))
            .back(5)
            .strafeRight(tile)
            .build();


    //Parking
    parkFromRedFront = drive.trajectorySequenceBuilder(redFrontCenterLine.end())
            .forward(tile * 1.6 - square_edge)
            .build();
    parkFromRedBack = drive.trajectorySequenceBuilder(redBackCenterLine.end())
            .forward(tile * 1.6 - square_edge)
            .build();

    parkFromBlueFront = drive.trajectorySequenceBuilder(blueBackCenterLine.end())
            .forward(tile * 3.5 - square_edge)
            .build();
    parkFromBlueBack = drive.trajectorySequenceBuilder(blueFrontCenterLine.end())
            .forward(tile * 3.5 - square_edge)
            .build();


    //Yellow Pixel
    redFrontYellowPixel = drive.trajectorySequenceBuilder(parkFromRedFront.end())
            .strafeLeft(0.5 * tile)
            .build();
    redBackYellowPixel = drive.trajectorySequenceBuilder(parkFromRedBack.end())
            .strafeRight(0.5 * tile)
            .build();
    blueFrontYellowPixel = drive.trajectorySequenceBuilder(parkFromBlueFront.end())
            .strafeLeft(0.5 * tile)
            .build();
    blueBackYellowPixel = drive.trajectorySequenceBuilder(parkFromBlueBack.end())
            .strafeRight(0.5 * tile)
            .build();

}
}
