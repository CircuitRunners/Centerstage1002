package org.firstinspires.ftc.teamcode.controllers.commands.intake;

import static org.firstinspires.ftc.teamcode.controllers.Constants.IntakeCommandConstants.DETECTION_THRESHOLD;
import static org.firstinspires.ftc.teamcode.controllers.commands.intake.IntakeStackCommand.FINISH_LOWSPEED_THRESHOLD;
import static org.firstinspires.ftc.teamcode.controllers.commands.intake.IntakeStackCommand.OUTTAKE_TIME_ROBOT;
import static org.firstinspires.ftc.teamcode.controllers.commands.intake.IntakeStackCommand.REQUIRED_TIME_MS;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Claw;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Intake;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Sensors;

@Photon
@Config
public class IntakeCommandEx extends CommandBase {
    private ElapsedTime runtime;
    private ElapsedTime intakeTimer, waitTimer; // Timer for the intake process
    private Intake intake;
    public Sensors sensors;
    private Claw claw;
    private int pixelsDetectedState = 0;
    private Intake.IntakePowers power;

    public IntakeCommandEx(HardwareMap hardwareMap, Claw claw, Intake intake, Intake.IntakePowers power) {
        this.intake = intake;
        this.claw = claw;
        this.sensors = new Sensors(hardwareMap);
        this.runtime = new ElapsedTime();
        this.intakeTimer = new ElapsedTime();
        this.waitTimer = new ElapsedTime();
        this.power = power;

        addRequirements(claw, intake);
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
        switch (pixelsDetectedState) {
            case 0: // Pixel not detected
                intake.setPower(power);
                if (sensors.getBottomDistance() < DETECTION_THRESHOLD && sensors.getTopDistance() < DETECTION_THRESHOLD) {
                    intakeTimer.reset();
                    pixelsDetectedState = 1;
                }
                break;
            case 1: // Pixel detected, timer running
                if (intakeTimer.milliseconds() < REQUIRED_TIME_MS) {

                    intake.setPower(power);
                    if (sensors.getBottomDistance() > DETECTION_THRESHOLD && sensors.getTopDistance() > DETECTION_THRESHOLD) {
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

    public double getIntakeCurrent () {
        return intake.getCurrent();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            intake.setPower(0);
        }
        claw.close(); // Close the claw at the end of the command
    }
}