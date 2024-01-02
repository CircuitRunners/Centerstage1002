package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends SubsystemBase {
    private ServoImplEx leftServo;
    private ServoImplEx rightServo;

    // THIS ONLY CALCULATES REGRESSION AND OFFSET!
    // CHANGE ACTUAL VALUES IN THE ARM POSITIONS ENUM BELOW!
    private static double[][] servoPositions = {
        {0.03, 0.13}, // DOWN positions (left, right)
        {0.64, 0.74}, // TRANSPORT positions (left, right)
        {0.7, 0.8}    // SCORING positions (left, right)
    };

    static double[] regressionResult = linearRegression(servoPositions);
    static double slope = regressionResult[0], intercept = regressionResult[1];

    public enum ArmPositions {
        DOWN(0.87),
        FIT(0.93),
        TRANSPORT(0.90), // This secures in places
        SCORING(0.37);

        public double position;

        ArmPositions(double position){
            this.position = position;
        }

        public double getRightPosition() {
            return this.position;
        }
    }

    public Arm(HardwareMap hardwareMap){
        leftServo = hardwareMap.get(ServoImplEx.class, "leftArm");
        rightServo = hardwareMap.get(ServoImplEx.class, "rightArm");

        forceDown();
    }

    // Motion profile constraints
    // TODO EDIT THESE
    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1.6, 1.8);

//    PROCESSORS

    // Motion profile, initialize to start at down
    private TrapezoidProfile armProfile = new TrapezoidProfile(
            constraints,
            new TrapezoidProfile.State(ArmPositions.DOWN.position, 0),
            new TrapezoidProfile.State(ArmPositions.DOWN.position, 0)
    );

    private ElapsedTime timer = new ElapsedTime();
    private double prevTarget = ArmPositions.DOWN.position;

    @Override
    public void periodic(){
        if(!armProfile.isFinished(timer.seconds())) {
            // Read the current target for the profile
            double leftPosition = armProfile.calculate(timer.seconds()).position;
            double rightPosition = leftPosition; //* slope + intercept; // Calculate right position using regression

            // Set servo positions according to the profile
            leftServo.setPosition(leftPosition);
            rightServo.setPosition(rightPosition);
        }
    }

    // Set the servos to a numerical position
    public void setPosition(double target) {
        // Create a new profile starting from the last position command
        if(prevTarget != target){
            armProfile = new TrapezoidProfile(
                    constraints,
                    new TrapezoidProfile.State(target, 0),
                    new TrapezoidProfile.State(leftServo.getPosition(), 0)
            );

            //Reset the timer
            timer.reset();
        }
        prevTarget = target;
    }

    public double[] getServoPositions (HardwareMap hardwareMap) {
        AnalogInput leftAnalogInput = hardwareMap.get(AnalogInput.class, "leftArmAnalog");
        AnalogInput rightAnalogInput = hardwareMap.get(AnalogInput.class, "rightArmAnalog");

        return new double[]{
            analogInputToPosition(leftAnalogInput),
            analogInputToPosition(rightAnalogInput),
        };
    }

    public double analogInputToPosition (AnalogInput analogInput) {
        double position = analogInput.getVoltage() / 3.3 * 360;
        return position;
    }

    public static double[] linearRegression(double[][] dataPoints) {
        int n = dataPoints.length;
        double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;

        for (double[] point : dataPoints) {
            sumX += point[0]; // Sum of left servo positions
            sumY += point[1]; // Sum of right servo positions
            sumXY += point[0] * point[1]; // Sum of product of left and right positions
            sumXX += point[0] * point[0]; // Sum of square of left positions
        }

        double slope = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
        double intercept = (sumY - slope * sumX) / n;

        return new double[]{slope, intercept};
    }


//    SETTERS

    // All the way to the rest position
    public void down(){
        setPosition(ArmPositions.DOWN.position);
    }

    public void up(){
        setPosition(ArmPositions.SCORING.position);
    }

    public void fit(){
        setPosition(ArmPositions.FIT.position);
    }

    public void transport () { setPosition(ArmPositions.TRANSPORT.position);}

    // Bypasses the profile
    public void forceDown(){
        leftServo.setPosition(ArmPositions.DOWN.position);
        rightServo.setPosition(ArmPositions.DOWN.getRightPosition());
    }

    // Goes to a position
    public void toPosition(ArmPositions level){
        setPosition(level.position);
    }

    public void toPosition(double level){
        setPosition(level);
    }

    // Disabling the servos to start tuning
    public void tuningModeOn() {
        leftServo.setPwmDisable();
        rightServo.setPwmDisable();
    }

    // Enables the servos to stop tuning
    public void tuningModeOff() {
        leftServo.setPwmEnable();
        rightServo.setPwmEnable();
    }

//    GETTERS
    public void forceSet (ArmPositions positions) {
        leftServo.setPosition(positions.position);
        rightServo.setPosition(positions.position);
    }

    // Helper method to log the current positions of the servos to telemetry
    public String getState() {
        leftServo.setPwmEnable();
        rightServo.setPwmEnable();
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);

        String theReturnStuff = String.format("L: %s\nR: %s", leftServo.getPosition(), rightServo.getPosition());

        leftServo.setPwmDisable();
        rightServo.setPwmDisable();
        return theReturnStuff;
    }

    public double getLeftPosition(){
        return leftServo.getPosition();
    }

    public double getRightPosition(){
        return rightServo.getPosition();
    }

    public String getRegressionResults() {
        return String.format("Best fit line: slope = %s, intercept = %s", slope, intercept);
    }
}