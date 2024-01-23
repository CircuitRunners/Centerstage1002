package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
//import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
//import org.firstinspires.ftc.teamcode.rr05.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.rr05.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

// Complete!
@Autonomous (name="Parking Auto (BLUE****, Stage)")
public class RedStage extends CommandOpMode {

//    private double powerFullMultiplier = DynamicConstants.multiplier;
    private MecanumDrive drive;

    private TeamPropDetector detector;
    private int locationID = 1; // set to center by default

    private Claw claw;

    private static Pose2d right= new Pose2d(12, -61.5, Math.toRadians(90));
    private static Pose2d left= new Pose2d(-36, -61.5, Math.toRadians(90));

    private DcMotorEx intake;
    @Override
    public void initialize(){

        //detector = new TeamPropDetector(hardwareMap, true);
        schedule(new BulkCacheCommand(hardwareMap));
        claw = new Claw(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        drive = new MecanumDrive(hardwareMap, right);

        Action rightPark = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(59.70, -61.30))
                .strafeTo(new Vector2d(35.63, -61.30))
                .strafeTo(new Vector2d(35.90, -36.30))
                .strafeTo(new Vector2d(50.61, -36.30))
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

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                rightPark
                        ),
                        new InstantAction(
                                claw::open
                        )

//                        new ParallelCommandGroup(
//                                new TrajectorySequenceCommand(drive, backOff),
////                                new InstantCommand(() -> {
////                                    intake.setPower(-0.6);
////                                })
//                        ),
//
//                        new WaitCommand(5000),
//                        new InstantCommand(() -> {
//                            intake.setPower(0);
//                        })
                )
        );
    };

}