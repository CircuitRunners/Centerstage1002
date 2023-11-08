package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;

import android.icu.util.ULocale;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

// Complete! :) [who needs I&R anyways?]
@Autonomous (name="Parking Auto (Blue, Stage)")
public class BlueStage extends CommandOpMode {

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

        detector = new TeamPropDetector(hardwareMap, true);
        schedule(new BulkCacheCommand(hardwareMap));


        intake = hardwareMap.get(DcMotorEx.class, "intake");
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        detector.startStream();
        while(opModeInInit()){
            locationID = detector.update();
            telemetry.addLine("Ready for start!");
            telemetry.addData("Prop", locationID);
            telemetry.update();
        }

        detector.stopStream();

        switch(locationID) {
            case 0: // Left
                schedule(new TrajectorySequenceCommand(drive, TrajectorySequences.blueBackLeftLine));
            case 1: // Middle
                schedule(new TrajectorySequenceCommand(drive, TrajectorySequences.blueBackCenterLine));
            case 2: // Right
                schedule(new TrajectorySequenceCommand(drive, TrajectorySequences.blueBackRightLine));
        };

        schedule(new TrajectorySequenceCommand(drive, TrajectorySequences.parkFromBlueBack));
        schedule(new TrajectorySequenceCommand(drive, TrajectorySequences.blueBackYellowPixel));
    };

}