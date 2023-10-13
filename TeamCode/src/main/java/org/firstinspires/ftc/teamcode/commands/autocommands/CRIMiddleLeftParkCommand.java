package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.presets.RetractOuttakeCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.vision.BeaconDetector;

public class CRIMiddleLeftParkCommand extends ParallelCommandGroup {


    public CRIMiddleLeftParkCommand(SampleMecanumDrive drive, Lift lift, Arm arm, Claw claw,
                                    BeaconDetector.BeaconTags tag, boolean isLeft, boolean isHigh) {

        addCommands(
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new InstantCommand(() -> {
                            lift.setLiftPower(0.8);
                        }),
                        new WaitCommand(500),
                        new InstantCommand(() -> {
                            lift.setLiftPower(0);
                        }),

//                        new WaitCommand(800),
                        new RetractOuttakeCommand(lift, arm, claw)
                )

        );

        switch (tag) {
            case CENTER:
                addCommands(
                        new TrajectorySequenceCommand(drive,
                                (isHigh) ?
                                        (isLeft) ? ThreeCycleTrajectories.leftHighLeftPark : ThreeCycleTrajectories.rightHighRightPark :
                                        (isLeft) ? ThreeCycleTrajectories.leftLeftPark : ThreeCycleTrajectories.rightRightPark
                        )
                );
                break;
            case RIGHT:
                addCommands(
                        new TrajectorySequenceCommand(drive,
                                (isHigh) ?
                                        (isLeft) ? ThreeCycleTrajectories.leftHighMiddlePark : ThreeCycleTrajectories.rightHighMiddlePark :
                                        (isLeft) ? ThreeCycleTrajectories.leftMiddlePark : ThreeCycleTrajectories.rightMiddlePark
                        )
                );
                break;
            case LEFT:
                addCommands(
                        new TrajectorySequenceCommand(drive,
                                (isHigh) ?
                                        (isLeft) ? ThreeCycleTrajectories.leftHighRightPark : ThreeCycleTrajectories.rightHighLeftPark :
                                        (isLeft) ? ThreeCycleTrajectories.leftRightPark : ThreeCycleTrajectories.rightLeftPark
                        )
                );
                break;
        }

    }
}
