package org.firstinspires.ftc.teamcode.controllers.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Pivot extends SubsystemBase {
    private ServoImplEx pivotServo;

    public enum PivotPositions {
        // (left, right)
        LEFTEX(0.28),
        RIGHTEX(0.72),

        LEFT(0.32),
        CENTER(0.49),
        RIGHT(0.68);


        private final double position;

        PivotPositions(double position_in) {
            this.position = position_in;
        }
    }

    public Pivot(HardwareMap hardwareMap){
        pivotServo = hardwareMap.get(ServoImplEx.class, "rotation");

        forceCenter();
    }

    // Motion profile constraints
    // TODO EDIT THESE
    public static TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1.5, 4.2);

//    PROCESSORS

    // Motion profile, initialize to start at down
    private TrapezoidProfile pivotProfile = new TrapezoidProfile(
            constraints,
            new TrapezoidProfile.State(PivotPositions.CENTER.position, 0),
            new TrapezoidProfile.State(PivotPositions.CENTER.position, 0)
    );

    private ElapsedTime timer = new ElapsedTime();

    private double prevTarget = PivotPositions.CENTER.position;

    @Override
    public void periodic(){
        if(!pivotProfile.isFinished(timer.seconds())) {
            // Read the current target for the profile
            double newPosition = pivotProfile.calculate(timer.seconds()).position;

            // Set servo positions according to the profile
            pivotServo.setPosition(newPosition);
        }
    }

    // Set the servos to a numerical position
    public void setPosition(PivotPositions target) {
//        setPosition(target.position);
        pivotServo.setPosition(target.position);
    }

    private void setPosition (double target) {
        // Create a new profile starting from the last position command
        if(prevTarget != target){
            pivotProfile = new TrapezoidProfile(
                    constraints,
                    new TrapezoidProfile.State(target, 0),
                    new TrapezoidProfile.State(pivotServo.getPosition(), 0)
            );

            //Reset the timer
            timer.reset();
        }
        prevTarget = target;
    }

    public double analogInputToPosition (AnalogInput analogInput) {
        double position = analogInput.getVoltage() / 3.3 * 360;
        return position;
    }

//    SETTERS

    // All the way to the rest position
    public void left(){
        setPosition(PivotPositions.LEFT);
    }
    public void leftEx() {setPosition(PivotPositions.LEFTEX);}

    public void center(){
        setPosition(PivotPositions.CENTER);
    }

    public void right(){
        setPosition(PivotPositions.RIGHT);
    }

    public void rightEx(){
        setPosition(PivotPositions.RIGHTEX);
    }

    // Bypasses the profile
    public void forceCenter(){
        pivotServo.setPosition(PivotPositions.CENTER.position);
    }

    public void toPosition(double level){
        setPosition(level);
    }

//    GETTERS
    public void forceSet (PivotPositions position) {
        pivotServo.setPosition(position.position);
    }

    public double getPosition(){
        return pivotServo.getPosition();
    }
}