package org.firstinspires.ftc.teamcode.commands.presets

import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.Arm
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftPositionCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.subsystems.Arm.ArmPositions
import org.firstinspires.ftc.teamcode.subsystems.Lift.LiftPositions
import org.firstinspires.ftc.teamcode.subsystems.Transfer

class MoveToScoringCommand(lift: Lift, arm: Arm, claw: Transfer, preset: Presets) : ParallelCommandGroup() {

    enum class Presets {
        SHORT,
        MID,
        HIGH
    }

    init {
        addCommands(
            ParallelCommandGroup(
                InstantCommand({
                    claw.close()
                }),
                when (preset) {
                    Presets.SHORT ->
                        LiftPositionCommand(lift, LiftPositions.SHORT.position, true)
                    Presets.MID ->
                        LiftPositionCommand(lift, LiftPositions.MID.position, true)
                    Presets.HIGH ->
                        LiftPositionCommand(lift, LiftPositions.HIGH.position, true)

                },
                SequentialCommandGroup(
                    WaitCommand(650),
                    InstantCommand({
                        arm.up()
                    }),
                )
            )
        )
        addRequirements(lift)
    }
}