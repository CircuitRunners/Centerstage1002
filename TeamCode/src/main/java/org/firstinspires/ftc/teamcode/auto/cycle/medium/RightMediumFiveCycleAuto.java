package org.firstinspires.ftc.teamcode.auto.cycle.medium;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.DropPoleCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.DropPreloadCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.GoToStackCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.ParkCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.ThreeCycleTrajectories;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.vision.BeaconDetector;


@Autonomous(name = "Right 1+5 M", group = "Medium")
public class RightMediumFiveCycleAuto extends CommandOpMode {


    private SampleMecanumDrive drive;
    private Claw claw;
    private Lift lift;
    private Arm arm;


    private BeaconDetector beaconDetector;
    private BeaconDetector.BeaconTags beaconId = BeaconDetector.BeaconTags.LEFT;

    @Override
    public void initialize() {
//        PhotonCore.enable();
        schedule(new BulkCacheCommand(hardwareMap));

        drive = new SampleMecanumDrive(hardwareMap);
        claw = new Claw(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);

        drive.setPoseEstimate(ThreeCycleTrajectories.rightStartingPosition);
        ThreeCycleTrajectories.clawOpenAction = () -> {
            claw.open();
            claw.poleGuideDown();
        };
        ThreeCycleTrajectories.clawCloseAction = () -> claw.close();
        ThreeCycleTrajectories.generateTrajectories(drive);
        claw.close();

        beaconDetector = new BeaconDetector(hardwareMap, false);


        DropPreloadCommand preloadCommand = new DropPreloadCommand(drive, lift, claw, arm, false);
        GoToStackCommand goToStackCommand1 =
                new GoToStackCommand(drive, lift, claw, arm, false, false, 1);
        GoToStackCommand goToStackCommand2 =
                new GoToStackCommand(drive, lift, claw, arm, false, false,  2);
        GoToStackCommand goToStackCommand3 =
                new GoToStackCommand(drive, lift, claw, arm, false, false, 3);
        GoToStackCommand goToStackCommand4 =
                new GoToStackCommand(drive, lift, claw, arm, false, false, 4);
        GoToStackCommand goToStackCommand5 =
                new GoToStackCommand(drive, lift, claw, arm, false, false, 5);
        DropPoleCommand dropPoleCommand1 = new DropPoleCommand(drive, lift, claw, arm, false, false);
        DropPoleCommand dropPoleCommand2 = new DropPoleCommand(drive, lift, claw, arm, false, false);
        DropPoleCommand dropPoleCommand3 = new DropPoleCommand(drive, lift, claw, arm, false, false);
        DropPoleCommand dropPoleCommand4 = new DropPoleCommand(drive, lift, claw, arm, false, false);
        DropPoleCommand dropPoleCommand5 = new DropPoleCommand(drive, lift, claw, arm, false, false);

        //Start vision
        beaconDetector.startStream();

        while (!isStarted()) {
            beaconId = beaconDetector.update();
            telemetry.addLine("Ready for start!");
            telemetry.addData("Beacon", beaconId);
            telemetry.update();
        }

        beaconDetector.stopStream();

        schedule(new SequentialCommandGroup(
                preloadCommand,
                goToStackCommand1,
                dropPoleCommand1,
                goToStackCommand2,
                dropPoleCommand2,
                goToStackCommand3,
                dropPoleCommand3,
                goToStackCommand4,
                dropPoleCommand4,
                goToStackCommand5,
                dropPoleCommand5,
                new ParkCommand(drive, lift, arm, claw, beaconId, false, false)
        ));
    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();

        telemetry.addData("lift", lift.getLiftPosition());
        telemetry.update();
    }
}
