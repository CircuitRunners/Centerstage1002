package org.firstinspires.ftc.teamcode.controllers.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Arm extends SubsystemBase {
    private ServoImplEx leftServo;
    private ServoImplEx rightServo;
    private Claw claw;

    private boolean usingClaw = false;

    public enum ArmPositions {
        // (left, right)
        MEGADOWN(.28,.28),
        DOWN(.26, .26), // right 0.22 before
//        DOWN(.4, .4), // right 0.22 before (this line overriten by new push
        SCORING(.83, .83); // 0.9 0.333

        private final double position_right;
        private final double position_left;

        ArmPositions(double position_left, double position_right) {
            this.position_right = position_right;
            this.position_left = position_left;
        }

        public double getRightPosition() {
            return this.position_right;
        }

        public double getLeftPosition() {
            return this.position_left;
        }
    }

    public Arm(HardwareMap hardwareMap){
        leftServo = hardwareMap.get(ServoImplEx.class, "leftArm");
        rightServo = hardwareMap.get(ServoImplEx.class, "rightArm");

        forceDown();
    }

    public Arm(HardwareMap hardwareMap, Claw claw){
        this(hardwareMap);

        usingClaw = true;

        this.claw = claw;
    }

    // Motion profile constraints
    // TODO EDIT THESE
    public static TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1.5, 4.2);

//    PROCESSORS

    // Motion profile, initialize to start at down
    private TrapezoidProfile leftArmProfile = new TrapezoidProfile(
            constraints,
            new TrapezoidProfile.State(ArmPositions.DOWN.position_left, 0),
            new TrapezoidProfile.State(ArmPositions.DOWN.position_left, 0)
    );

    private TrapezoidProfile rightArmProfile = new TrapezoidProfile(
            constraints,
            new TrapezoidProfile.State(ArmPositions.DOWN.position_right, 0),
            new TrapezoidProfile.State(ArmPositions.DOWN.position_right, 0)
    );

    private ElapsedTime timer = new ElapsedTime();

    private double prevTargetLeft = ArmPositions.DOWN.position_left;
    private double prevTargetRight = ArmPositions.DOWN.position_right;

    @Override
    public void periodic(){
        if(!leftArmProfile.isFinished(timer.seconds())) {
            // Read the current target for the profile
            double leftPosition = leftArmProfile.calculate(timer.seconds()).position;
            double rightPosition = rightArmProfile.calculate(timer.seconds()).position;

            // Set servo positions according to the profile
            leftServo.setPosition(leftPosition);
            rightServo.setPosition(rightPosition);
        }
    }

    // Set the servos to a numerical position
    public void setPosition(ArmPositions target) {
        setLeftPosition(target.position_left);
        setRightPosition(target.position_right);
    }

    public void setPosition (double forced) {
        leftServo.setPosition(forced);
        rightServo.setPosition(forced);
    }

    private void setLeftPosition (double target) {
        // Create a new profile starting from the last position command
        if(prevTargetLeft != target){
            leftArmProfile = new TrapezoidProfile(
                    constraints,
                    new TrapezoidProfile.State(target, 0),
                    new TrapezoidProfile.State(leftServo.getPosition(), 0)
            );

            //Reset the timer
            timer.reset();
        }
        prevTargetLeft = target;
    }

    private void setRightPosition (double target) {
        // Create a new profile starting from the last position command
        if(prevTargetRight != target){
            rightArmProfile = new TrapezoidProfile(
                    constraints,
                    new TrapezoidProfile.State(target, 0),
                    new TrapezoidProfile.State(rightServo.getPosition(), 0)
            );

            //Reset the timer
            timer.reset();
        }
        prevTargetRight = target;
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

//    SETTERS

    // All the way to the rest position
    public void down(){
        setPosition(ArmPositions.DOWN);
//        if (usingClaw) {
//            claw.open();
//        }
    }

    public void up(){
        setPosition(ArmPositions.SCORING);
//        if (usingClaw) {
//            claw.open();
//        }
    }

    // Bypasses the profile
    public void forceDown(){
        leftServo.setPosition(ArmPositions.MEGADOWN.position_left);
        rightServo.setPosition(ArmPositions.MEGADOWN.position_right);
    }

    public void setClawAutoDisable () {
        usingClaw = false;
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
    public void forceSet (ArmPositions position) {
        leftServo.setPosition(position.position_left);
        rightServo.setPosition(position.position_right);
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
}