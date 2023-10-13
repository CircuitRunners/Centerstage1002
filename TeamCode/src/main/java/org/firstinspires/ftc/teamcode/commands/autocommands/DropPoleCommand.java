package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class DropPoleCommand extends ParallelCommandGroup {


    public DropPoleCommand(SampleMecanumDrive drive, Lift lift, Claw claw, Arm arm, boolean isLeft,
                           boolean isHigh) {

        addCommands(

                new SequentialCommandGroup(
                        new WaitCommand(200),
                        new InstantCommand(() -> arm.setPosition(
                               Arm.ArmPositions.SCORING.position
                        )),
                        new WaitCommand(1050),
                        new InstantCommand(claw::primePoleGuide)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(300),
                        (isHigh) ?
                                new TrajectorySequenceCommand(
                                        drive, isLeft ? ThreeCycleTrajectories.leftHighToPole : ThreeCycleTrajectories.rightHighToPole
                                ) :
                                new TrajectorySequenceCommand(
                                        drive, isLeft ? ThreeCycleTrajectories.leftToPole : ThreeCycleTrajectories.rightToPole
                                )
//                        new WaitCommand(50)
                ),
                new SequentialCommandGroup(
//                        new WaitCommand(100),
                        new ParallelRaceGroup(
                                new LiftPositionCommand(lift,
                                        (isHigh) ? Lift.LiftPositions.HIGH.position : Lift.LiftPositions.MID.position,
                                        true
                                ),
                                new WaitCommand(1900)
                        )
                )
        );
    }


}
