package org.firstinspires.ftc.teamcode.controllers.commands.lift;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.controllers.subsytems.Lift;

@Config
public class LiftPositionCommand extends CommandBase {

    private PIDFController liftController;
    public static PIDCoefficients coefficients =
            new PIDCoefficients(0.02, 0.000, 0.000);
    // 0.0269, 0.003, 0.0014

    public static double tolerance = 15;
    private double targetPosition = 0;
    private boolean holdAtEnd;
    private final Lift lift;

    public static double setpointPos;
    double liftPosition = 0;
    double liftVelocity = 0;
    double controllerOutput = 0;

    public LiftPositionCommand(Lift lift, int targetPosition, boolean holdAtEnd){
        addRequirements(lift);

        this.holdAtEnd = holdAtEnd;
        this.lift = lift;
        this.targetPosition = targetPosition;
        setpointPos = targetPosition;

        //Add a feedforward term to counteract gravity
        liftController = new PIDFController(coefficients, 0.0, 0.0, 0.00, (x, v) -> {
            double kG = 0;
            if (liftPosition < 850) kG = 0.09;
            else if (liftPosition < 1380) kG = 0.11;
            else kG = 0.13;

            return kG * lift.getVoltageComp();
        });
        liftController.setOutputBounds(-0.7, 1.0); // gravity CA
    }
    @Override
    public void initialize(){
        //once
        liftController.reset();
        liftController.setTargetPosition(targetPosition);
    }

    //Run repeatedly while the command is active
    @Override
    public void execute(){
        liftPosition = lift.getLiftPosition();
        liftVelocity = lift.getLiftVelocity();

        liftController.setTargetPosition(targetPosition);

        controllerOutput = liftController.update(liftPosition, liftVelocity);

        //Update the lift power with the controller
        lift.setLiftPower(controllerOutput);
    }

    @Override
    public boolean isFinished(){
        //End if the lift position is within the tolerance
        return Math.abs(targetPosition - liftPosition) <= tolerance;
    }

    @Override
    public void end(boolean interrupted){
        if (holdAtEnd) lift.setLiftPower(0.09);
        else lift.brake_power();
    }

}
