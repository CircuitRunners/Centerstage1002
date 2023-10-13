package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;


import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class GoToStackCommand extends ParallelCommandGroup {


    public GoToStackCommand(SampleMecanumDrive drive, Lift lift, Claw claw, Arm arm, boolean isLeft,
                            boolean isHigh, int cycle) {


        // Have the liftPos set to something relevant in case something goes wonky
        int liftPos = 161;

        switch (cycle) {
            case 1:
                liftPos = 161;
                break;
            case 2:
                liftPos = 126;
                break;
            case 3:
                liftPos = 86;
                break;
            case 4:
                liftPos = 42;
                break;
            case 5:
                liftPos = 2;
                break;
        }
        addCommands(
                new SequentialCommandGroup(
                        (cycle != 1) ?
                                (cycle >= 4) ? new TrajectorySequenceCommand(
                                        drive, (isHigh) ?
                                        isLeft ? ThreeCycleTrajectories.leftHighToStackDrifted : ThreeCycleTrajectories.rightHighToStackDrifted :
                                        isLeft ? ThreeCycleTrajectories.leftToStackDrifted : ThreeCycleTrajectories.rightToStackDrifted

                                ) :
                                        new TrajectorySequenceCommand(
                                                drive, (isHigh) ?
                                                isLeft ? ThreeCycleTrajectories.leftHighToStack : ThreeCycleTrajectories.rightHighToStack :
                                                isLeft ? ThreeCycleTrajectories.leftToStack : ThreeCycleTrajectories.rightToStack
                                        )
                                : new TrajectorySequenceCommand(
                                drive, isLeft ? ThreeCycleTrajectories.leftToStackPreload : ThreeCycleTrajectories.rightToStackPreload
                        )
//                        new WaitCommand(100)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(800),
                        new InstantCommand(claw::sheathPoleGuide),
                        new WaitCommand(200),
                        new InstantCommand(claw::fullOpen)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new InstantCommand(() -> arm.setLevel(Arm.ArmPositions.DOWN))
                ),
                new SequentialCommandGroup(
                        new WaitCommand(600),
                        new ParallelRaceGroup(
                                new LiftPositionCommand(lift, liftPos, true),
                                new WaitCommand(1800)
                        )
                )
        );
    }

}
