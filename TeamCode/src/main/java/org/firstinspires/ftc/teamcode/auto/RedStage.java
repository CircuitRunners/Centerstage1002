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
import com.outoftheboxrobotics.photoncore.Photon;
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
//@Photon
@Autonomous (name="Parking Auto (BLUE****, Stage)")
public class RedStage extends CommandOpMode {

//    private double powerFullMultiplier = DynamicConstants.multiplier;
    private MecanumDrive drive;

    private TeamPropDetector detector;
    private int locationID = 1; // set to center by default

    private Claw claw;

    private static Pose2d right= new Pose2d(61.5, 12, Math.toRadians(0));
    private static Pose2d left= new Pose2d(-36, -61.5, Math.toRadians(90));

    private DcMotorEx intake;
    @Override
    public void initialize(){
        schedule(new BulkCacheCommand(hardwareMap));
        detector = new TeamPropDetector(hardwareMap, true);
        claw = new Claw(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        drive = new MecanumDrive(hardwareMap, right);

        detector.startStream();


        Action rightParkTrajectory = drive.actionBuilder(right)
                .splineToConstantHeading(new Vector2d(33, 12), Math.toRadians(0))
                .turn(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(33, 48), Math.toRadians(-90))

//                .splineTo(new Vector2d(yToX(-32.97), xToY(10.77)) , Math.toRadians(91.08))
//                .splineTo(new Vector2d(yToX(-32.97), xToY(53.02)) , Math.toRadians(0.00))
                .build();


//                .strafeTo(new Vector2d(-61.30,-59.70))
//                .strafeTo(new Vector2d(-61.30, -35.63))
//                .strafeTo(new Vector2d( -36.30,-35.90))
//                .strafeTo(new Vector2d(-36.30,-50.61))


        Action rightBackboardTrajectory = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(12, -33.89))
                .strafeTo(new Vector2d(19.59, -39.11))
                .strafeToSplineHeading(new Vector2d(49.94, -36.03), Math.toRadians(180.00))
                .strafeTo(new Vector2d(-59.57, -36.57))
                .strafeTo(new Vector2d(49.94, -35.90))
                .build();


//        detector.startStream();
        while(opModeInInit()){
            // Show when auto good
            telemetry.addLine("Ready for start!");
            locationID = detector.update();

            telemetry.addData("Prop", locationID);
            telemetry.update();
            sleep(50);
        }


        switch (locationID) {
            case 0: {

            }
            case 1: {

            }
            case 2: {

            }
        }
        Actions.runBlocking(rightParkTrajectory);
//                new ParallelAction(
//                        new SequentialAction(
//                                rightBackboardTrajectory
//                        ),
//                        new InstantAction(
//                                claw::open
//                        )
//
////                        new ParallelCommandGroup(
////                                new TrajectorySequenceCommand(drive, backOff),
//////                                new InstantCommand(() -> {
//////                                    intake.setPower(-0.6);
//////                                })
////                        ),
////
////                        new WaitCommand(5000),
////                        new InstantCommand(() -> {
////                            intake.setPower(0);
////                        })
//                )
//        );

    }

    public double xToY (double xVal) {
        if (Math.signum(xVal) == -1) {
            return Math.abs(xVal);
        } else {
            return -Math.abs(xVal);
        }
    }
    public double yToX (double yVal) {
        if (Math.signum(yVal) == -1) {
            return -Math.abs(yVal);
        } else {
            return Math.abs(yVal);
        }
    }
}

