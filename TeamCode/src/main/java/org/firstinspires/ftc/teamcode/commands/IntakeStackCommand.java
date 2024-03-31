package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoArm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Photon
@Config
public class IntakeStackCommand extends CommandBase {
    private ElapsedTime runtime;
    private ElapsedTime intakeTimer, waitTimer; // Timer for the intake process
    private Intake intake;
    public DistanceSensor distanceSensor;
    public DistanceSensor distanceSensorTop;
    private Claw claw;
    private int pixelsDetectedState = 0;
    private Intake.IntakePowers power;
    public static double DETECTION_THRESHOLD = 4.0; // Threshold for the distance sensor

    public static double MOTOR_CURRENT_THRESHOLD = 8; //
    // consider reducing if need faster cycle times
    public static double REQUIRED_TIME_MS = 200; // Required time in milliseconds,

    public static double FINISH_LOWSPEED_THRESHOLD = 400;
    public static double OUTTAKE_TIME_ROBOT = 1350;

    public IntakeStackCommand(HardwareMap hardwareMap, Claw claw, Intake intake, Intake.IntakePowers power) {
        this.intake = intake;
        this.claw = claw;
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        this.distanceSensorTop = hardwareMap.get(DistanceSensor.class, "topDistanceSensor");
        this.runtime = new ElapsedTime();
        this.intakeTimer = new ElapsedTime();
        this.waitTimer = new ElapsedTime();
        this.power = power;

        addRequirements(claw, intake); // If using Command-based structure, declare subsystem dependencies.
    }

    @Override
    public void initialize() {
        claw.open();
        pixelsDetectedState = 0;
        runtime.reset();
        intakeTimer.reset();
        intake.setPower(power);
    }

    @Override
    public void execute() {

//        switch (pixelsDetectedState) {
//            case 0: // Pixel not detected
//                intake.setPower(power);
//                if (distanceSensor.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD) {
//                    intakeTimer.reset();
//                    pixelsDetectedState = 1;
//                }
//                break;
//            case 1: // Pixel detected, timer running
//                if (intakeTimer.milliseconds() < REQUIRED_TIME_MS) {
//                    intake.setPower(power);
//                    if (distanceSensor.getDistance(DistanceUnit.CM) > DETECTION_THRESHOLD) {
//                        pixelsDetectedState = 0; // Reset if the distance goes above the threshold
//                    }
//                } else {
//                    pixelsDetectedState = 2; // Time required has passed, and pixel is consistently detected
//                    waitTimer.reset();
//                }
//                break;
//            case 2: // Pixel intake process is complete
//                claw.close();
//                if (waitTimer.milliseconds() < FINISH_LOWSPEED_THRESHOLD) {
//                    intake.setPower(Intake.IntakePowers.SLOW);
//                } else if (waitTimer.milliseconds() < OUTTAKE_TIME_ROBOT) {
//                    intake.setPower(Intake.OuttakePowers.NORMAL);
//                } else {
//                    intake.setPower(0);
//                    pixelsDetectedState = 3;
//                }
//                break;
//        }

        // use motor current power as an additional criteria for auto outtake
        switch (pixelsDetectedState) {
            case 0: // Pixel not detected
                intake.setPower(power);
                if (distanceSensor.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD && distanceSensorTop.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD) {
                    intakeTimer.reset();
                    pixelsDetectedState = 1;
                }
                else if (intake.getCurrent() > MOTOR_CURRENT_THRESHOLD) {
                    pixelsDetectedState = 2;
                }
                break;
            case 1: // Pixel detected, timer running
                if (intakeTimer.milliseconds() < REQUIRED_TIME_MS && intake.getCurrent() > MOTOR_CURRENT_THRESHOLD) {

                    intake.setPower(power);
                    if (distanceSensor.getDistance(DistanceUnit.CM) > DETECTION_THRESHOLD && distanceSensorTop.getDistance(DistanceUnit.CM) > DETECTION_THRESHOLD) {
                        pixelsDetectedState = 0; // Reset if the distance goes above the threshold
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

    }

    @Override
    public boolean isFinished() {
        return pixelsDetectedState == 3;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            intake.setPower(0);
        }
        claw.close(); // Close the claw at the end of the command
    }
}