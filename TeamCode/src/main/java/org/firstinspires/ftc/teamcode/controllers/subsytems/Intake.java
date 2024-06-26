package org.firstinspires.ftc.teamcode.controllers.subsytems;

import static org.firstinspires.ftc.teamcode.controllers.common.utilities.Utilities.squash;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// Certified Validated!
@Config
public class Intake extends SubsystemBase {
    public static double STOPPED_POWER = 0;
    private DcMotorEx intakeMotor;
    private ElapsedTime runtime;
    private double runDuration;
    private boolean isRunning;

    public interface PowerSetting {
        double getSpeed();
    }

    public enum IntakePowers implements PowerSetting {
        FAST(-1), // really fast but enough to grip
        NORMAL(-1), // normal
        SLOW(-0.35); // slow

        public final double speed;

        IntakePowers(double speed){
            this.speed = speed;
        }

        @Override
        public double getSpeed() {
            return speed;
        }
    }

    public enum OuttakePowers implements PowerSetting {
        FAST(-IntakePowers.FAST.speed), // really fast but enough to grip
        NORMAL(-IntakePowers.NORMAL.speed), // normal
        SLOW(-IntakePowers.SLOW.speed); // slow

        public final double speed;

        OuttakePowers(double speed){
            this.speed = speed;
        }

        @Override
        public double getSpeed() {
            return speed;
        }
    }

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
//        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        runtime = new ElapsedTime();
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void setPower(double powerLeft, double powerRight) {
        setPower(STOPPED_POWER);
        // Set dynamic powers!
        powerRight = squash(powerRight);
        powerLeft = squash(powerLeft);

        if (powerRight > powerLeft) {
            setPower(1);
        } else if (powerLeft > powerRight) {
            setPower(-1);
        }
    }

    public void setPower(PowerSetting power) {
        intakeMotor.setPower(power.getSpeed());
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    public double getCurrent() { return intakeMotor.getCurrent(CurrentUnit.AMPS); }

    public void runForDuration(double duration, double speed) {
        setPower(speed);
        runDuration = duration;
        runtime.reset();
        isRunning = true;
    }

    public void runForDuration(double duration, PowerSetting speed) {
        setPower(speed);
        runDuration = duration;
        runtime.reset();
        isRunning = true;
    }


    public void intakeForDuration(double duration) {
        setPower(IntakePowers.NORMAL);
        runDuration = duration;
        runtime.reset();
        isRunning = true;
    }

    public void outtakeForDuration(double duration) {
        setPower(OuttakePowers.NORMAL);
        runDuration = duration;
        runtime.reset();
        isRunning = true;
    }

    public void testingMode(double power) {
        setPower(power);
    }

    public int getCurrentPosition () {
        return intakeMotor.getCurrentPosition();
    }

    @Override
    public void periodic() {
        if (isRunning && runtime.seconds() > runDuration) {
            setPower(STOPPED_POWER);
            isRunning = false;
        }
    }
}