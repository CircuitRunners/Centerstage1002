package org.firstinspires.ftc.teamcode.commands.presets;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

//Brings everything to rest position
public class RetractOuttakeCommand extends ParallelCommandGroup {

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