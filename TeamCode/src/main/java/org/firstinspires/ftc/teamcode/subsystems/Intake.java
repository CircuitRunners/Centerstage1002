package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utilities.Utilities.CLOSE_TO_ONE;
import static org.firstinspires.ftc.teamcode.utilities.Utilities.CLOSE_TO_ZERO;
import static org.firstinspires.ftc.teamcode.utilities.Utilities.DEBOUNCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.utilities.Utilities.squash;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake extends SubsystemBase {
    private DcMotorEx intakeMotor;
    private ElapsedTime runtime;
    private double runDuration;
    private boolean isRunning;

    public interface PowerSetting {
        double getSpeed();
    }

    public enum IntakePowers implements PowerSetting {
        FAST(0.72), // really fast but enough to grip
        NORMAL(0.18), // normal
        SLOW(0.18); // slow

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
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        runtime = new ElapsedTime();
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void setPower(double powerLeft, double powerRight) {
        // Set dynamic powers!
        powerRight = squash(powerRight);
        powerLeft = squash(powerLeft);

        // deb from one is ~ 0.85
        // This means HARD right press
        if (powerRight > CLOSE_TO_ONE) {
            if (powerLeft > 0.5)
                setPower(IntakePowers.NORMAL);
            else {
                setPower(IntakePowers.FAST);
            }
        }
        else if (powerLeft > CLOSE_TO_ONE) {
            if (powerRight > 0.5)
                setPower(OuttakePowers.NORMAL);
            else {
                setPower(OuttakePowers.FAST);
            }
        } else if (powerRight > CLOSE_TO_ZERO) {
            setPower(IntakePowers.NORMAL);
        } else if (powerLeft > CLOSE_TO_ZERO) {
            setPower(OuttakePowers.NORMAL);
        } else if (
                (powerRight > DEBOUNCE_THRESHOLD)
        ) {
            setPower(IntakePowers.SLOW);
        } else if (
                (powerLeft > DEBOUNCE_THRESHOLD)
        ) {
            setPower(OuttakePowers.SLOW);
        }
    }

    public void setPower(PowerSetting power) {
        intakeMotor.setPower(power.getSpeed());
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

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

    @Override
    public void periodic() {
        if (isRunning && runtime.seconds() > runDuration) {
            setPower(0);
            isRunning = false;
        }
    }
}