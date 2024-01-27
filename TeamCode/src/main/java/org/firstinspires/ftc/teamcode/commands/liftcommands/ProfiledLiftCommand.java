package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
public class ProfiledLiftCommand extends CommandBase {

    private PIDFController liftController;
    private MotionProfile profile;
    private ElapsedTime timer = new ElapsedTime();

    public static PIDCoefficients coefficients =
            new PIDCoefficients(0.02, 0.000, 0.000); // Adjust PID coefficients as needed

    private double tolerance = 15;
    private boolean holdAtEnd;
    private final Lift lift;
    private final double targetPosition;

    public static double setpointPos;
    public static double liftPosition = 0;
    public static double liftVelocity = 0;
    public static double controllerOutput = 0;

    public ProfiledLiftCommand(Lift lift, int targetPosition, boolean holdAtEnd){
        addRequirements(lift);

        this.holdAtEnd = holdAtEnd;
        this.lift = lift;
        this.targetPosition = targetPosition;
        setpointPos = targetPosition;

        // Gravity feedforward term to counteract gravity
        liftController = new PIDFController(coefficients, 0.0, 0.0, 0.00, (x, v) -> {
            double kG = 0;
            if (liftPosition < 850) kG = 0.09;
            else if (liftPosition < 1380) kG = 0.11;
            else kG = 0.13;

            return kG * lift.getVoltageComp();
        });
        liftController.setOutputBounds(-0.7, 1.0);
    }

    public ProfiledLiftCommand(Lift lift, int targetPosition, boolean holdAtEnd, boolean strict){
        addRequirements(lift);

        if (strict) tolerance = 10;

        this.holdAtEnd = holdAtEnd;
        this.lift = lift;
        this.targetPosition = targetPosition;
        setpointPos = targetPosition;

        // Gravity feedforward term to counteract gravity
        liftController = new PIDFController(coefficients, 0.0, 0.0, 0.00, (x, v) -> {
            double kG = 0;
            if (liftPosition < 850) kG = 0.09;
            else if (liftPosition < 1380) kG = 0.11;
            else kG = 0.13;

            return kG * lift.getVoltageComp();
        });
        liftController.setOutputBounds(-0.7, 1.0);
    }

    @Override
    public void initialize(){
        liftController.reset();

        // Generate the motion profile
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(lift.getLiftPosition(), lift.getLiftVelocity()),
                new MotionState(targetPosition, 0),
                // These values are placeholders; adjust maxVel, maxAccel, and maxJerk as needed
                2500, // maxVel 700
                3250, // maxAccel 820
                0    // maxJerk
        );

        timer.reset();
    }

    @Override
    public void execute(){
        liftPosition = lift.getLiftPosition();
        liftVelocity = lift.getLiftVelocity();
        double currentTime = timer.seconds();
        MotionState state = profile.get(currentTime);

        // Update the PIDFController with the profile's state
        liftController.setTargetPosition(state.getX());
        liftController.setTargetVelocity(state.getV());
        liftController.setTargetAcceleration(state.getA());

        controllerOutput = liftController.update(liftPosition, liftVelocity);

        // Update the lift power with the controller
        lift.setLiftPower(controllerOutput);

        setpointPos = state.getX();
        // Additional setpoint variables can be set here if needed
    }

    @Override
    public boolean isFinished(){
        // End if the lift position is within the tolerance
        return Math.abs(targetPosition - liftPosition) <= tolerance;
    }

    @Override
    public void end(boolean interrupted){
        if (holdAtEnd) lift.setLiftPower(0.09);
        else lift.brake_power(); // Assuming brake_power() is a method to stop the lift
    }
}