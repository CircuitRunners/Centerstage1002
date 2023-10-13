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
public class ProfiledLiftPositionCommand extends CommandBase {

    private PIDFController liftController;
    private MotionProfile profile;


    public static PIDCoefficients coefficients =
            new PIDCoefficients(0.0269, 0.0045, 0.00138);//p=0.0268, i=0.005, d=0.00145
    public static double kV = 0.0019;
    public static double kA = 0.0;
    public static double kStatic = 0.03;

    private double tolerance = 5;
    private boolean holdAtEnd;
    private final Lift lift;
    private final double targetPosition;

    private double liftPosition;

    public static double setpointPos = 0;
    public static double setpointVel = 0;
    public static double setpointPosError = 0;
    public static double setpointAccel = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public ProfiledLiftPositionCommand(Lift lift, double targetPosition, boolean holdAtEnd) {
        this.holdAtEnd = holdAtEnd;
        this.lift = lift;
        this.targetPosition = targetPosition;

        addRequirements(lift);


        liftController = new PIDFController(coefficients, kV * lift.getVoltageComp(), kA, kStatic, (x, v) -> {
            double kG;
            if (liftPosition < 283) kG = 0.18;
            else if (liftPosition < 580) kG = 0.196;
            else kG = 0.222;

            return kG * lift.getVoltageComp();
        });
        liftController.setOutputBounds(-0.45, 0.98);
    }


    @Override
    public void initialize() {



        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(lift.getLiftPosition(), lift.getLiftVelocity()),
                new MotionState(targetPosition, 0),
                700,
                820,
                0
        );

        timer.reset();
        liftController.reset();
    }

    @Override
    public void execute() {
        liftPosition = lift.getLiftPosition();
        double currentVelo = lift.getLiftVelocity();
        double currentTime = timer.seconds();
//        TrapezoidProfile.State state = profile.calculate(currentTime);
        MotionState state = profile.get(currentTime);


        //Update the real controller target
        liftController.setTargetPosition(state.getX());
        liftController.setTargetVelocity(state.getV());
        liftController.setTargetAcceleration(state.getA());


        //Get the controller output
        double controllerOutput = liftController.update(liftPosition, currentVelo);

        //Update the lift power with the controller
        lift.setLiftPower(controllerOutput);

        setpointPos = state.getX();
        setpointVel = state.getV();
        setpointAccel = state.getA();
        setpointPosError = targetPosition - liftPosition;
    }

    @Override
    public boolean isFinished() {
        //End if the lift position is within the tolerance
        return Math.abs(targetPosition - liftPosition) <= tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        if (holdAtEnd) lift.setLiftPower(0.2);
        else lift.stop();
    }



}
