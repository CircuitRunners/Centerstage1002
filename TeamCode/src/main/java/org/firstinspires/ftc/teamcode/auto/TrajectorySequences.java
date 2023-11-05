package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectorySequences {

    private double powerFullMultiplier = DynamicConstants.multiplier;
    private SampleMecanumDrive drive;

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
    TrajectorySequence redBackCenterLine = drive.trajectorySequenceBuilder(back_red)
            .forward(1.25 * tile - square_edge)
            .back(1.1 * tile)
            .turn(Math.toRadians(-90))
            .build();

    TrajectorySequence redBackrightLine = drive.trajectorySequenceBuilder(back_red)
            .splineTo(new Vector2d(14, -35), Math.toRadians(0))
            .back(4)
            .strafeRight(tile)
            .forward(2)
            .build();

    TrajectorySequence redBackleftLine = drive.trajectorySequenceBuilder(back_red)
            .splineTo(new Vector2d(9, -35), Math.toRadians(180))
            .back(4)
            .strafeLeft(tile)
            .turn(Math.toRadians(-180))
            .back(2)
            .build();



    //RedFront
    TrajectorySequence redFrontCenterLine = drive.trajectorySequenceBuilder(front_red)
            .forward(1.25 * tile - square_edge)
            .back(2)
            .strafeLeft(0.5 * tile)
            .forward(tile)
            .turn(Math.toRadians(-90))
            .forward(0.5*tile)
            .build();

    TrajectorySequence redFrontRightLine = drive.trajectorySequenceBuilder(front_red)
            .splineTo(new Vector2d(-32, -35), Math.toRadians(0))
            .back(5)
            .strafeLeft(tile)
            .build();

    TrajectorySequence redFrontLeftLine = drive.trajectorySequenceBuilder(front_red)
            .splineTo(new Vector2d(-38, -35), Math.toRadians(180))
            .back(5)
            .strafeRight(tile)
            .build();



    //BlueBack
    TrajectorySequence blueBackCenterLine = drive.trajectorySequenceBuilder(back_blue)
            .forward(1.25 * tile - square_edge)
            .back(1.1 * tile)
            .turn(Math.toRadians(90))
            .build();

    TrajectorySequence blueBackrightLine = drive.trajectorySequenceBuilder(back_blue)
            .splineTo(new Vector2d(9, 35), Math.toRadians(180))
            .back(4)
            .strafeRight(tile)
            .turn(Math.toRadians(180))
            .back(2)
            .build();

    TrajectorySequence blueBackleftLine = drive.trajectorySequenceBuilder(back_blue)
            .splineTo(new Vector2d(14, 35), Math.toRadians(0))
            .back(4)
            .strafeLeft(tile)
            .forward(2)
            .build();




    //BlueFront
    TrajectorySequence blueFrontCenterLine = drive.trajectorySequenceBuilder(front_blue)
            .forward(1.25 * tile - square_edge)
            .back(2)
            .strafeRight(0.5 * tile)
            .forward(tile)
            .turn(Math.toRadians(90))
            .forward(0.5*tile)
            .build();


    TrajectorySequence blueFrontrightLine = drive.trajectorySequenceBuilder(front_blue)
            .splineTo(new Vector2d(-38, 34), Math.toRadians(180))
            .back(2)
            .strafeLeft(tile)
            .build();

    TrajectorySequence blueFrontleftLine = drive.trajectorySequenceBuilder(front_blue)
            .splineTo(new Vector2d(-32, 35), Math.toRadians(0))
            .back(5)
            .strafeRight(tile)
            .build();




    //Parking
    TrajectorySequence parkFromBack = drive.trajectorySequenceBuilder(back_red)
            .forward(tile * 1.6 - square_edge)
            .build();

    TrajectorySequence parkFromFront = drive.trajectorySequenceBuilder(front_red)
            .forward(tile * 3.5 - square_edge)
            .build();

    

    //Green Pixel
    TrajectorySequence rightGreenPixel = drive.trajectorySequenceBuilder(back_red)
            .strafeLeft(0.5*tile)
            .build();
    TrajectorySequence leftGreenPixel = drive.trajectorySequenceBuilder(front_red)
            .strafeLeft(0.5*tile)
            .build();

}
