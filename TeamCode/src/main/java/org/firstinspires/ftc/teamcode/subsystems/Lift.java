package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Lift extends SubsystemBase {
    public enum LiftPositions {
        DOWN(0),
        SHORT(200),
        MID(450),
        HIGH(770);

        public int position;

        LiftPositions(int position){
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }
    }

    public double winchServoInit = 0.5, winchServoEngage = 1.0;

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private DcMotorEx winchMotor;
    private ServoImplEx winchServo;

    private VoltageSensor voltageSensor;
    private double voltageComp = 1.0;

    public Lift(HardwareMap hardwareMap){

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLift");
        winchMotor = hardwareMap.get(DcMotorEx.class, "winch");

        // Zero both the motors
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // We don't need to use builtin PID
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Negate the gravity when stopped
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // change to brake if bad
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageComp = 12.0 / voltageSensor.getVoltage();
    }

    @Override
    public void periodic(){
        // happens every loop
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
        // THIS IS NEGATED BECAUSE THE VALUES ARE ALL NEGATIVE
        return leftMotor.getCurrentPosition();
    }

    public double getLiftVelocity(){
        return leftMotor.getVelocity();
    }

    public boolean atUpperLimit(){
        return getLiftPosition() > 2550;
    }

    public boolean atLowerLimit(){
        return getLiftPosition() < 3;
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
        setLiftPower(power);
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
