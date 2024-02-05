package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.CommandBase;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

// IF LIFT IS BAD REMOVE PHOTON!!! IMPOPRTANTE
@Photon
@Config
public class HeroicLiftCommand extends CommandBase {
    FtcDashboard dashboard;

    private PIDFController liftController;
    private MotionProfile profile;
    private ElapsedTime timer = new ElapsedTime();

    public double kP = 0.02, kI = 0.000, kD = 0.000;

    private PIDCoefficients coefficients = new PIDCoefficients(kP, kI, kD); // Adjust PID coefficients as needed

    // Feedforward Coefficients
    private double kV = 0.0, kA = 0.0, kStatic = 0.00;

    // The tolerance for getting to a certain position. Strict tries to get just a bit closer.
    private double LIFT_POSITION_TOLERANCE = 15,
            LIFT_POSITION_TOLERANCE_STRICT = 10;

    private double liftPosition = 0;
    private double liftVelocity = 0;
    private double controllerOutput = 0;

    final double MOTION_PROFILE_MAX_VELOCITY = 2500,
            MOTION_PROFILE_MAX_ACCEL = 3250,
            MOTION_PROFILE_MAX_JERK = 5000; //todo TUNE THIS

    final double GRAVITY_FEEDFORWARD_COMPENSATION_FIRST_STAGE = 0.09,
            GRAVITY_FEEDFORWARD_COMPENSATION_SECOND_STAGE = 0.11,
            GRAVITY_FEEDFORWARD_COMPENSATION_THIRD_STAGE = 0.13;
    final double LIFT_FIRST_STAGE_POSITION_TICKS = 850,
            LIFT_SECOND_STAGE_POSITION_TICKS = 1380;

    Telemetry telem;

    boolean holdAtEnd;
    final Lift lift;
    final double targetPosition;

    double setPointPos;

    public HeroicLiftCommand(Lift lift, int targetPosition, boolean holdAtEnd){
        this(lift, targetPosition, holdAtEnd, false);
    }

    public HeroicLiftCommand(Lift lift, int targetPosition, boolean holdAtEnd, boolean strict){
        addRequirements(lift);

        if (strict) this.LIFT_POSITION_TOLERANCE = LIFT_POSITION_TOLERANCE_STRICT;

        this.holdAtEnd = holdAtEnd;
        this.lift = lift;
        this.targetPosition = targetPosition;

        setPointPos = targetPosition;

        // Gravity feedforward term to counteract gravity
        liftController = new PIDFController(coefficients, kV, kA, kStatic, (x, v) -> {
            // Feedforward Gravitational Below
            double kG = 0;
            if (liftPosition < LIFT_FIRST_STAGE_POSITION_TICKS) {
                kG = GRAVITY_FEEDFORWARD_COMPENSATION_FIRST_STAGE;
            }
            else if (liftPosition < LIFT_SECOND_STAGE_POSITION_TICKS) {
                kG = GRAVITY_FEEDFORWARD_COMPENSATION_SECOND_STAGE;
            }
            else {
                kG = GRAVITY_FEEDFORWARD_COMPENSATION_THIRD_STAGE;
            }

            return kG * lift.getVoltageComp();
        });

        // Prevent it from going down TOO fast
        // This is the same as the maximum amount of kG compensation subtracted from max negative value
        liftController.setOutputBounds(-0.87, 1.0);
    }

    @Override
    public void initialize(){
        dashboard = FtcDashboard.getInstance();
        telem = dashboard.getTelemetry();
        liftController.reset();

        // Generate the motion profile
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(lift.getLiftPosition(), lift.getLiftVelocity(), 0, 0),
                new MotionState(targetPosition, 0, 0, 0),
                MOTION_PROFILE_MAX_VELOCITY,
                MOTION_PROFILE_MAX_ACCEL,
                MOTION_PROFILE_MAX_JERK
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

        // Additional SetPoint variables can be set here if needed
        setPointPos = state.getX();


// Example of sending telemetry data
        telem.addData("Lift Position", liftPosition);
        telem.addData("Lift Velocity", liftVelocity);
        telem.update();
    }

    @Override
    public boolean isFinished(){
        // End if the lift position is within the tolerance
        return Math.abs(targetPosition - liftPosition) <= LIFT_POSITION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted){
        if (holdAtEnd) lift.setLiftPower(0.09);
        else lift.brake_power(); // Assuming brake_power() is a method to stop the lift
    }
}