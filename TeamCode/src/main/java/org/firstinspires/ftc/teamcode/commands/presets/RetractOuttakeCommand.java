package org.firstinspires.ftc.teamcode.commands.presets;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

//Brings everything to rest position
public class RetractOuttakeCommand extends ParallelCommandGroup {

    public RetractOuttakeCommand(Lift lift, Arm arm, Claw claw){
        addCommands(
                new LiftPositionCommand(lift, Lift.LiftPositions.DOWN.position, false),
                new InstantCommand(arm::down),
                new InstantCommand(claw::open),
                new InstantCommand(claw::sheathPoleGuide)
        );

        addRequirements(lift);
    }

}
