package org.firstinspires.ftc.teamcode.commands.presets

import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.Arm
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftCommand
import org.firstinspires.ftc.teamcode.subsystems.Lift.LiftPositions
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Pivot

class MoveToScoringCommandEx(lift: Lift, arm: Arm, claw: Claw, preset: Presets, pivot: Pivot) : ParallelCommandGroup() {

    enum class Presets {
        BOTTOM,
        SHORT,
        MID,
        HIGH
    }

    init {
        addCommands(
            ParallelCommandGroup(
                SequentialCommandGroup(
                    // Change this ms to change when the arm comes up
                    InstantCommand(pivot::center),
                    WaitCommand(20),
                    InstantCommand({
                        arm.up()
                    }),
                ),
                InstantCommand({
                    claw.close()
                }),
                when (preset) {
                    Presets.SHORT ->
                        ProfiledLiftCommand(lift, LiftPositions.SHORT.position, true)
                    Presets.MID ->
                        ProfiledLiftCommand(lift, LiftPositions.MID.position, true)
                    Presets.HIGH ->
                        ProfiledLiftCommand(lift, LiftPositions.HIGH.position, true)
                    Presets.BOTTOM ->
                        ProfiledLiftCommand(lift, 300, true);
                }
            )
        )
        addRequirements(lift)
    }
}