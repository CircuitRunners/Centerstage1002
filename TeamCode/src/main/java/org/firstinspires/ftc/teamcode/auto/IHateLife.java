package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.halfPI;
import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.rotationConstant;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
//import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
//import org.firstinspires.ftc.teamcode.rr05.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.rr05.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.commands.presets.MoveToScoringCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoArm;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

// Complete!
//@Photon
@Autonomous (name="Parking Auto (IHateLife, Stage)")
public class IHateLife extends CommandOpMode {

    //    private double powerFullMultiplier = DynamicConstants.multiplier;
    private MecanumDrive drive;

//    private TeamPropDetector detector;
//    private int locationID = 1; // set to center by default

    private Claw claw;
    private ExtendoArm extendoArm;
    private Action purpleTrajectory;
    private Lift lift;
    private MoveToScoringCommand moveToScoringCommand;
    private Arm arm;

    private static Pose2d right= new Pose2d(61.5, 9, Math.toRadians(0));
    private static Pose2d left= new Pose2d(-36, -61.5, Math.toRadians(90));

    private DcMotorEx intake;
    @Override
    public void initialize(){
        schedule(new BulkCacheCommand(hardwareMap));

//        detector = new TeamPropDetector(hardwareMap, true);
        claw = new Claw(hardwareMap);
        extendoArm = new ExtendoArm(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        Pose2d startPos = new Pose2d(61.5, 9, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, startPos);

        arm = new Arm(hardwareMap);
        lift = new Lift(hardwareMap);
        moveToScoringCommand = new MoveToScoringCommand(lift,arm,claw, MoveToScoringCommand.Presets.SHORT);

        purpleTrajectory = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(32.5, 9), Math.toRadians(0))
                .build();


//                .splineToConstantHeading(new Vector2d(33, 12), Math.toRadians(0))
//                .turn(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(33, 48), Math.toRadians(-90))
//
////                .splineTo(new Vector2d(yToX(-32.97), xToY(10.77)) , Math.toRadians(91.08))
////                .splineTo(new Vector2d(yToX(-32.97), xToY(53.02)) , Math.toRadians(0.00))
//                .build();


//                .strafeTo(new Vector2d(-61.30,-59.70))
//                .strafeTo(new Vector2d(-61.30, -35.63))
//                .strafeTo(new Vector2d( -36.30,-35.90))
//                .strafeTo(new Vector2d(-36.30,-50.61))



//        detector.startStream();
        while(opModeInInit()){
            // Show when auto good
            telemetry.addLine("Ready for start!");
//            locationID = detector.update();

//            telemetry.addData("Prop", locationID);
            telemetry.update();
            sleep(50);
        }

        Actions.runBlocking(
                purpleTrajectory
        );

    }

}

