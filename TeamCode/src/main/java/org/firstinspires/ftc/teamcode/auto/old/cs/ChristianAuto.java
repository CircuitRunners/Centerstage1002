package org.firstinspires.ftc.teamcode.auto.old.cs;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.DynamicConstants;
import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BeaconDetector;


@Autonomous (name="Christian's Amazing Autonomous Sequential Command Op Mode")
public class ChristianAuto extends CommandOpMode {

    private double powerFullMultiplier = DynamicConstants.multiplier;
    private SampleMecanumDrive drive;

    private BeaconDetector beaconDetector;
    private BeaconDetector.BeaconTags beaconId = BeaconDetector.BeaconTags.LEFT;

    private Pose2d startPose = new Pose2d(0, 0, toRadians(0.0));
    private static double tile = 24;
    private static double half_tile = 12;
    private static double robot_len = 9;
    private static double square_edge = 1.5;

    private static Pose2d right= new Pose2d(12, -61.5, Math.toRadians(90));
    private static Pose2d left= new Pose2d(-36, -61.5, Math.toRadians(90));

    @Override
    public void initialize(){
        schedule(new BulkCacheCommand(hardwareMap));

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(right)
            .strafeRight(powerFullMultiplier*(tile * 2 - square_edge))
            .build();
        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(left)
                .splineToConstantHeading(new Vector2d(-24, -12), Math.toRadians(0))
                .strafeRight(tile * 3.5 - square_edge)
                .build();

        while(opModeInInit()){
//            beaconId = beaconDetector.update();
//            telemetry.addLine("Ready for start!");
//            telemetry.addData("Beacon", beaconId);
            telemetry.update();
        }

//        beaconDetector.stopStream();

        //Gets much more complex, for now simply switch
//        switch(beaconId){
//            case LEFT:
//                schedule(new TrajectorySequenceCommand(drive, leftTrajectoryAbs));
//                break;
//            case CENTER:
//                schedule(new TrajectorySequenceCommand(drive, middleTrajectoryAbs));
//                break;
//            case RIGHT:
//                schedule(new TrajectorySequenceCommand(drive, rightTrajectoryAbs));
//                break;
//        }

        schedule(new TrajectorySequenceCommand(drive, leftPark));
    }



}