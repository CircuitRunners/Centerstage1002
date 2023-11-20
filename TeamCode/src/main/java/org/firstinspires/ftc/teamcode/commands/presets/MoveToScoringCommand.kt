package org.firstinspires.ftc.teamcode.commands.presets

import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.Arm
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftPositionCommand
import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.teamcode.subsystems.Arm.ArmPositions
import org.firstinspires.ftc.teamcode.subsystems.Lift.LiftPositions

class MoveToScoringCommand(lift: Lift, arm: Arm, preset: Presets) : ParallelCommandGroup() {

    enum class Presets {
        DOWN,
        SHORT,
        MID,
        HIGH
    }

    init {
        addCommands(
                when (preset) {
                    Presets.DOWN ->
                        LiftPositionCommand(lift, LiftPositions.DOWN.position, true)
                    Presets.SHORT ->
                        LiftPositionCommand(lift, LiftPositions.SHORT.position, true)
                    Presets.MID ->
                        LiftPositionCommand(lift, LiftPositions.MID.position, true)
                    Presets.HIGH ->
                        LiftPositionCommand(lift, LiftPositions.HIGH.position, true)

                },

//                InstantCommand({
//                    when (preset) {
//                        Presets.DOWN -> arm.position = ArmPositions.GROUND.position
//                        else -> arm.position = ArmPositions.SCORING.position
//                    }
//                }),

                InstantCommand({
//                    if(claw.clawPosition == Claw.ClawPosition.FULL_OPEN) claw.close()
                }),
        )
        addRequirements(lift)
    }
}