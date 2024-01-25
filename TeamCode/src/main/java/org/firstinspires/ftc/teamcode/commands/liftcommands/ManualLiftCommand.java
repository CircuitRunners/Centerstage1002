package org.firstinspires.ftc.teamcode.commands.liftcommands;

import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.circle;
import static org.firstinspires.ftc.teamcode.utilities.CrossBindings.square;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class ManualLiftCommand extends CommandBase {

    private final Lift lift;
    private final GamepadEx manipulator;

    private final double up = 1.0;
    private final double down = -0.7;

    private final double slowUp = 0.57;
    private final double slowDown = -0.36;

    public ManualLiftCommand(Lift lift, GamepadEx manipulator) {


        addRequirements(lift);

        this.lift = lift;
        this.manipulator = manipulator;

    }

    public boolean isManualActive() {
        return manipulator.getButton(GamepadKeys.Button.DPAD_UP) ||
                manipulator.getButton(GamepadKeys.Button.DPAD_DOWN);
    }

    @Override
    public void execute() {
        //Two dpad buttons cant be pressed at the same time so we don't have to worry about that.

        boolean slow = manipulator.getButton(square);

        //Check if the up button is pressed
        if (manipulator.getButton(GamepadKeys.Button.DPAD_UP) && !lift.atUpperLimit()) {
            lift.setLiftPower((slow) ? slowUp : up);
        }

        //Then check if the down is pressed
        else if (manipulator.getButton(GamepadKeys.Button.DPAD_DOWN) && !lift.atLowerLimit()) {

            if (lift.getLiftVelocity() < -500 && lift.getLiftPosition() < 350) {
                lift.setLiftPower(0);
            } else {
                lift.setLiftPower((slow) ? slowDown : down);
            }

        }

        //Otherwise, do nothing
        else {
            if (lift.getLiftPosition() < 10) lift.setLiftPower(0);
                // Counter motor being on close to the bottom
            else if (lift.getLiftPosition() < 40) lift.setLiftPower(-.05);
                // Slide 1 Counter
            else if (lift.getLiftPosition() < 850)
                lift.setLiftPower(0.09*lift.getVoltageComp()); //0.19 *
                // Slide 2 counter
            else if (lift.getLiftPosition() < 1380)
                lift.setLiftPower(0.10*lift.getVoltageComp()); //0.20 * lift.getVoltageComp()
                // Slide 3 counter
            else if (lift.getLiftPosition() < 1640)
                lift.setLiftPower(0.11*lift.getVoltageComp()); //0.20 * lift.getVoltageComp()
                // Other Counter
            else lift.setLiftPower(0.13*lift.getVoltageComp());
        }
    }
}

