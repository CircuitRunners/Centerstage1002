package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class ManualLiftCommand extends CommandBase {

    private final Lift lift;
    private final GamepadEx manipulator;

    private final double up = 1.0;
    private final double down = -0.7;

    private final double slowUp = 0.65;
    private final double slowDown = -0.01;

    public ManualLiftCommand(Lift lift, GamepadEx manipulator){


        addRequirements(lift);

        this.lift = lift;
        this.manipulator = manipulator;

    }

    public boolean isManualActive(){
        return manipulator.getButton(GamepadKeys.Button.DPAD_UP) ||
                manipulator.getButton(GamepadKeys.Button.DPAD_DOWN);
    }

    @Override
    public void execute() {
        //Two dpad buttons cant be pressed at the same time so we don't have to worry about that.

        boolean slow = manipulator.getButton(GamepadKeys.Button.X);


        //Check if the up button is pressed
        if(manipulator.getButton(GamepadKeys.Button.DPAD_UP) && !lift.atUpperLimit()){
            lift.setLiftPower((slow) ? slowUp : up);
        }

        //Then check if the down is pressed
        else if (manipulator.getButton(GamepadKeys.Button.DPAD_DOWN) && !lift.atLowerLimit()) {
            lift.setLiftPower((slow) ? slowDown : down);
        }

        //Otherwise, do nothing
//        else {
//            if(lift.getLiftPosition() < 3) lift.setLiftPower(1);
//            else if(lift.getLiftPosition() < 283) lift.setLiftPower(1); //0.19 * lift.getVoltageComp()
//            else if(lift.getLiftPosition() < 580) lift.setLiftPower(1); //0.20 * lift.getVoltageComp()
//            else lift.setLiftPower(1);
//        }
    }


}