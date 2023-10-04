package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Lift extends SubsystemBase {


    public enum LiftPositions {
        DOWN(0),
        SHORT(83),
        MID(376),
        HIGH(685);

        public int position;

        LiftPositions(int position){
            this.position = position;
        }
    }

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    private VoltageSensor voltageSensor;
    private double voltageComp = 1.0;

    public Lift(HardwareMap hardwareMap){

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLift");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageComp = 12.0 / voltageSensor.getVoltage();
    }

    @Override
    public void periodic(){
        //happens every loop
    }

    public void setLiftPower(double power){
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }

    public void stop(){
        setLiftPower(0);
    }

    public double getLiftPosition(){
        return leftMotor.getCurrentPosition();
    }

    public double getLiftVelocity(){
        return leftMotor.getVelocity();
    }

    public boolean atUpperLimit(){
        return getLiftPosition() > 830;
    }

    public boolean atLowerLimit(){
        return getLiftPosition() < 3;
    }

    public void resetLiftPosition(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getVoltageComp(){
        return voltageComp;
    }
}
