package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous (name="Parking Auto (Blue, Stage)")
public class BlueStage extends CommandOpMode {

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



        while(opModeInInit()){
//            beaconId = beaconDetector.update();
//            telemetry.addLine("Ready for start!");
//            telemetry.addData("Beacon", beaconId);
            telemetry.update();
        }

//        beaconDetector.stopStream();



        schedule(
            new SequentialCommandGroup(
                    switch(beaconId){
                        case LEFT:
                            schedule(new TrajectorySequenceCommand(drive, TrajectorySequences.blueBackLeftLine));
                            break;
                        case CENTER:
                            schedule(new TrajectorySequenceCommand(drive, TrajectorySequences.blueBackCenterLine));
                            break;
                        case RIGHT:
                            schedule(new TrajectorySequenceCommand(drive, TrajectorySequences.blueBackRightLine));
                            break;
                    }
                new TrajectorySequenceCommand(drive, TrajectorySequences.parkFromBlueBack);
                new TrajectorySequenceCommand(drive, TrajectorySequences.blueBackYellowPixel);
                    );

        );

    };

}