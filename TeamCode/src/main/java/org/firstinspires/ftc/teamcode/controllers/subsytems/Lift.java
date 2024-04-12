package org.firstinspires.ftc.teamcode.controllers.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class Lift extends SubsystemBase {
    public enum LiftPositions {
        DOWN(0),
        SHORT(203),
        MID(469),
        HIGH(953);

        public int position;

        LiftPositions(int position){
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }

        public void setPosition(int position) {
            this.position = position;
        }
    }

    public static double UPPER_LIMIT = 2300, LOWER_LIMIT = 50;
    public static double winchServoInit = 0.5, winchServoEngage = 0.66;

    public static int
            LIFTPOSITIONS_DOWN = LiftPositions.DOWN.position,
            LIFTPOSITIONS_SHORT = LiftPositions.SHORT.position,
            LIFTPOSITIONS_MID = LiftPositions.MID.position,
            LIFTPOSITIONS_HIGH = LiftPositions.HIGH.position;

    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx winchMotor;
    ServoImplEx winchServo;

    private VoltageSensor voltageSensor;
    private double voltageComp;
    public static double VOLTAGE_WHEN_LIFT_TUNED = 13.0;

    public Lift(HardwareMap hardwareMap){

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLift");
        winchMotor = hardwareMap.get(DcMotorEx.class, "winch");
        winchServo = hardwareMap.get(ServoImplEx.class, "hangServo");

        // Zero both the motors
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // We don't need to use builtin PID
        // THIS DOES NOT DISABLE THE ENCODER.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Make it so the when positive power applied, lift moves up.
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Negate the gravity when stopped
        //TODO gravity PID coefficients?
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // change to brake if bad
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageComp = VOLTAGE_WHEN_LIFT_TUNED / voltageSensor.getVoltage();
    }

    @Override
    public void periodic(){
        // happens every loop
        LiftPositions.HIGH.setPosition(LIFTPOSITIONS_HIGH);
        LiftPositions.MID.setPosition(LIFTPOSITIONS_MID);
        LiftPositions.SHORT.setPosition(LIFTPOSITIONS_SHORT);
        LiftPositions.DOWN.setPosition(LIFTPOSITIONS_DOWN);
    }

    public void setLiftPower(double power){
        //TODO this could be the PID demon
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void brake_power(){
        setLiftPower(0);
    }

    public double getLiftPosition(){
        return leftMotor.getCurrentPosition();
    }

    public double getLiftVelocity(){
        return leftMotor.getVelocity();
    }

    public boolean atUpperLimit(){
        return getLiftPosition() > UPPER_LIMIT;
    }

    public boolean atLowerLimit(){
        return getLiftPosition() < LOWER_LIMIT;
    }

    public void resetLiftPosition(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getVoltageComp(){
        return voltageComp;
    }

    public void hangPower (double power) {
//        setLiftPower(power);
        double constantWinchMultiplier = 1.0;
        winchMotor.setPower(power * constantWinchMultiplier);
    }
    
    public void enableHang () {
        winchServo.setPosition(winchServoEngage);
    }

    public void initialInitHang () {
        winchServo.setPosition(winchServoInit);
    }
}
