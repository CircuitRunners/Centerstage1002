package org.firstinspires.ftc.teamcode.commands.liftcommands;


import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class ManualLiftResetCommand extends CommandBase {
    private final Lift lift;
    private final GamepadEx manipulator;


    public ManualLiftResetCommand(Lift lift, GamepadEx manipulator){

        addRequirements(lift);

        this.lift = lift;
        this.manipulator = manipulator;
    }

    @Override
    public void execute(){
        if(manipulator.getButton(GamepadKeys.Button.DPAD_DOWN)){
            lift.setLiftPower(-0.2);
        } else {
            lift.brake();
        }
    }


    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        lift.brake();
        lift.resetLiftPosition();
    }
}