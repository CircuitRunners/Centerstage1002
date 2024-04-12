package org.firstinspires.ftc.teamcode.controllers.commands.presets;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.controllers.commands.lift.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Arm;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Claw;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Lift;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Pivot;

// Brings everything to rest position
public class RetractOuttakeCommand extends ParallelCommandGroup {

    public RetractOuttakeCommand(Lift lift, Arm arm, Claw claw, Pivot pivot) {
        addCommands(
                new SequentialCommandGroup(
                        new InstantCommand(arm::down),
                        new InstantCommand(pivot::center),
                        new LiftPositionCommand(lift, Lift.LiftPositions.DOWN.position, false)
//                    new InstantCommand(claw::open)
                )
//            new InstantCommand(claw::open)
        );
    }

    public RetractOuttakeCommand(Lift lift, Arm arm, Claw claw){
        addCommands(
            new SequentialCommandGroup(
                    new InstantCommand(arm::down),
                    new LiftPositionCommand(lift, Lift.LiftPositions.DOWN.position, false)
//                    new InstantCommand(claw::open)
            )
//            new InstantCommand(claw::open)
        );

        addRequirements(lift);
    }

}