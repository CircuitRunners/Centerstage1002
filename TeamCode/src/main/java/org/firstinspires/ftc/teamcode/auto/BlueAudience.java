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
import org.firstinspires.ftc.teamcode.rr05.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr05.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

// Complete! :) [who needs I&R anyways?]
@Autonomous (name="Parking Auto (Blue, Audience)")
public class BlueAudience extends CommandOpMode {

    private double powerFullMultiplier = DynamicConstants.multiplier;
    private SampleMecanumDrive drive;

    private TeamPropDetector detector;
    private int locationID = 1; // set to center by default

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

        //detector = new TeamPropDetector(hardwareMap, true);
        schedule(new BulkCacheCommand(hardwareMap));

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(right)
                .strafeLeft(powerFullMultiplier*(tile * 2 - square_edge))
                .build();

        // CHANGE THIS RIGHT VALUE THIS IS BAD
        TrajectorySequence backOff = drive.trajectorySequenceBuilder(right)
                .back(half_tile)
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(left)
                .forward(powerFullMultiplier*(square_edge + 2 * tile))
                .strafeLeft(powerFullMultiplier*(tile * 4 - square_edge))
                .build();

//        detector.startStream();
        while(opModeInInit()){
            //locationID = detector.update();
            //telemetry.addLine("Ready for start!");
            //telemetry.addData("Prop", locationID);
            telemetry.update();
        }

//        detector.stopStream();

//        //crazy 2 cycle purple yellow cycle
//        int thing = 1;
//        switch (thing){
//            case 0: //Left
//                schedule(new SequentialCommandGroup((new TrajectorySequenceCommand(drive, TrajectorySequences.blueBackLeftLine2Cycle))));
//            case 1: //Center
//                schedule(new SequentialCommandGroup((new TrajectorySequenceCommand(drive, TrajectorySequences.blueBackCenterLine2Cycle))));
//            case 2: //Right
//                schedule(new TrajectorySequenceCommand(drive, TrajectorySequences.blueBackRightLine2Cycle));
//        }

        //purple yellow auto
//        switch(locationID) {
//            case 0: // Left
//                schedule(new TrajectorySequenceCommand(drive, TrajectorySequences.blueBackLeftLine));
//                break;
//            case 1: // Middle
//                schedule(new TrajectorySequenceCommand(drive, TrajectorySequences.blueBackCenterLine));
//                break;
//            case 2: // Right
//                schedule(new TrajectorySequenceCommand(drive, TrajectorySequences.blueBackRightLine));
//                break;
//        };

//       schedule(new TrajectorySequenceCommand(drive, TrajectorySequences.parkFromBlueBack));
//        schedule(new TrajectorySequenceCommand(drive, TrajectorySequences.blueBackYellowPixel));

        schedule(
                new SequentialCommandGroup(
                        new TrajectorySequenceCommand(drive, leftPark),
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