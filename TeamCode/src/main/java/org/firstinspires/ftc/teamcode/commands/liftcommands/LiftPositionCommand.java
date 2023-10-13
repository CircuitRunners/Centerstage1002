package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
public class LiftPositionCommand extends CommandBase {

    private PIDFController liftController;
    public static PIDCoefficients coefficients =
            new PIDCoefficients(0.0269, 0.003, 0.0014); //ki=0.0045
    //Bottom 0.14, 0.145, 0.176

    private double tolerance = 3;
    private double targetPosition = 0;
    private boolean holdAtEnd;
    private final Lift lift;

    public static double setpointPos;
    public static double liftPosition = 0;
    public static double liftVelocity = 0;
    public static double controllerOutput = 0;

    public LiftPositionCommand(Lift lift, int targetPosition, boolean holdAtEnd){
        addRequirements(lift);


        this.holdAtEnd = holdAtEnd;
        this.lift = lift;
        this.targetPosition = targetPosition;
        setpointPos = targetPosition;

        //Add a feedforward term to counteract gravity
        liftController = new PIDFController(coefficients, 0.0, 0.0, 0.03, (x, v) -> {
            double kG;
            if (liftPosition < 283) kG = 0.18;
            else if (liftPosition < 580) kG = 0.196;
            else kG = 0.222;

            return kG * lift.getVoltageComp();
        });
        liftController.setOutputBounds(-0.65, 0.98);
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
        if (holdAtEnd) lift.setLiftPower(0.2);
        else lift.stop();
    }

}
