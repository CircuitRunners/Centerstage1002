package org.firstinspires.ftc.teamcode.controllers.common.utilities;

import com.arcrobotics.ftclib.command.CommandBase;

public class CommandRepeater extends CommandBase {
    private final CommandBase command;
    private final int repeatCount;
    private int currentCount = 0;

    public CommandRepeater(CommandBase command, int repeatCount) {
        this.command = command;
        this.repeatCount = repeatCount;
    }

    @Override
    public void initialize() {
        currentCount = 0;
    }

    @Override
    public void execute() {
        if (!command.isScheduled()) {
            if (currentCount < repeatCount) {
                command.schedule();
                currentCount++;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return currentCount >= repeatCount && !command.isScheduled();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            command.cancel();
        }
    }
}