package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BeaconDetector;


@Autonomous (name="Parking Auto (Red, Stage)")
public class RightAuto extends CommandOpMode {

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
    public void initialize(){
        schedule(new BulkCacheCommand(hardwareMap));


        intake = hardwareMap.get(DcMotorEx.class, "intake");
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(right)
                .strafeRight(powerFullMultiplier*(tile * 2 - square_edge))
                .build();

        // CHANGE THIS RIGHT VALUE THIS IS BAD
        TrajectorySequence backOff = drive.trajectorySequenceBuilder(right)
                .back(half_tile)
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(left)
                .forward(powerFullMultiplier*(square_edge + 2 * tile))
                .strafeRight(powerFullMultiplier*(tile * 4 - square_edge))
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

        schedule(
                new SequentialCommandGroup(
                        new TrajectorySequenceCommand(drive, rightPark),
                        new ParallelCommandGroup(
                                new TrajectorySequenceCommand(drive, backOff),
                                new InstantCommand(() -> {
                                    intake.setPower(-0.6);
                                })
                        ),

                        new WaitCommand(5000),
                        new InstantCommand(() -> {
                            intake.setPower(0);
                        })
                )
        );

    };



}