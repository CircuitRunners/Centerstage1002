package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Photon
public class IntakeCommandExMax extends CommandBase {
    private ElapsedTime runtime;
    private ElapsedTime intakeTimer; // Timer for the intake process
    private ElapsedTime waitTimer; // Timer for post-intake actions
    private Intake intake;
    private DistanceSensor distanceSensor;
    private Claw claw;
    private int pixelsDetectedState = 0;
    private Intake.IntakePowers power;
    private static final double DETECTION_THRESHOLD = 4.0; // Threshold for the distance sensor
    private static final double REQUIRED_TIME_MS = 900; // Required time in milliseconds
    private static final double FINISH_LOWSPEED_THRESHOLD = 400;
    private static final int STALL_THRESHOLD = 100; // Encoder counts threshold for stall detection
    private static final int STALL_RECOVERY_TIME = 150; // Time in milliseconds to reverse motor for unjamming
    private static final double OUTTAKE_TIME_ROBOT = 1500;
    private int lastEncoderPosition;

    public IntakeCommandExMax(HardwareMap hardwareMap, Claw claw, Intake intake, Intake.IntakePowers power) {
        this.intake = intake;
        this.claw = claw;
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        this.power = power;
        this.runtime = new ElapsedTime();
        this.intakeTimer = new ElapsedTime();
        this.waitTimer = new ElapsedTime();
        this.lastEncoderPosition = intake.getCurrentPosition();

        addRequirements(claw, intake); // If using Command-based structure, declare subsystem dependencies.
    }

    @Override
    public void initialize() {
        claw.open();
        pixelsDetectedState = 0;
        runtime.reset();
        intakeTimer.reset();
        waitTimer.reset();
        intake.setPower(power);
        lastEncoderPosition = intake.getCurrentPosition();
    }

    @Override
    public void execute() {
        int currentEncoderPosition = intake.getCurrentPosition();
        switch (pixelsDetectedState) {
            case 0: // Pixel not detected
                intake.setPower(power);
                if (distanceSensor.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD) {
                    intakeTimer.reset();
                    pixelsDetectedState = 1;
                }
                break;
            case 1: // Pixel detected, timer running
                if (intakeTimer.milliseconds() < REQUIRED_TIME_MS) {
                    intake.setPower(power);
                    if (distanceSensor.getDistance(DistanceUnit.CM) > DETECTION_THRESHOLD) {
                        pixelsDetectedState = 0; // Reset if the distance goes above the threshold
                    }
                    // Stall detection logic
                    if (Math.abs(currentEncoderPosition - lastEncoderPosition) < STALL_THRESHOLD) {
                        // Motor is stalled, attempt to unjam
                        intake.setPower(-power.speed); // Reverse motor briefly
                        waitTimer.reset();
                        while (waitTimer.milliseconds() < STALL_RECOVERY_TIME) {
                            // Wait for the motor to reverse and hopefully unjam
                        }
                        intake.setPower(power); // Resume normal operation
                        lastEncoderPosition = intake.getCurrentPosition(); // Update last position after handling stall
                    }
                } else {
                    pixelsDetectedState = 2; // Time required has passed, and pixel is consistently detected
                    waitTimer.reset();
                }
                break;
            case 2: // Pixel intake process is complete
                claw.close();
                if (waitTimer.milliseconds() < FINISH_LOWSPEED_THRESHOLD) {
                    intake.setPower(Intake.IntakePowers.SLOW);
                } else if (waitTimer.milliseconds() < OUTTAKE_TIME_ROBOT) {
                    intake.setPower(Intake.OuttakePowers.NORMAL);
                } else {
                    intake.setPower(0);
                    pixelsDetectedState = 3;
                }
                break;
        }
        lastEncoderPosition = currentEncoderPosition; // Update the last encoder position
    }

    @Override
    public boolean isFinished() {
        return pixelsDetectedState == 3;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0); // Stop the intake motor
        if (!interrupted) {
            claw.close(); // Close the claw at the end of the command if not interrupted
        }
    }
}