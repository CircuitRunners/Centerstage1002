package org.firstinspires.ftc.teamcode.controllers.commands.intake;

import static org.firstinspires.ftc.teamcode.controllers.Constants.IntakeCommandConstants.DETECTION_THRESHOLD;
import static org.firstinspires.ftc.teamcode.controllers.Constants.IntakeCommandConstants.FINISH_LOWSPEED_THRESHOLD;
import static org.firstinspires.ftc.teamcode.controllers.Constants.IntakeCommandConstants.OUTTAKE_TIME_ROBOT;
import static org.firstinspires.ftc.teamcode.controllers.Constants.IntakeCommandConstants.REQUIRED_TIME_MS;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controllers.subsytems.Arm;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Claw;
import org.firstinspires.ftc.teamcode.controllers.subsytems.ExtendoArm;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Intake;
//import org.firstinspires.ftc.teamcode.controllers.subsytems.Sensors;

@Photon
@Config
public class IntakeCommandStack extends CommandBase {
    private Arm arm;
    private ElapsedTime runtime;
    private ElapsedTime intakeTimer, waitTimer, lastUpdateTimer; // Timer for the intake process
    private Intake intake;
    //public Sensors sensors;
    private Claw claw;
    private int pixelsDetectedState = 0;
    private double lastUpdateTime = 0;
    private Intake.IntakePowers power;
    private static final double POSITION_INCREMENT_PER_MILLISECOND = 0.05 / 1000; // How much to lower the arm per second
    private ExtendoArm.ExtendoPositions extendo_pos;

    public IntakeCommandStack(HardwareMap hardwareMap, Claw claw, Intake intake, Arm arm, Intake.IntakePowers power, ExtendoArm.ExtendoPositions extendoPositions) {
        this.intake = intake;
        this.claw = claw;
        this.arm = arm;
        //this.sensors = new Sensors(hardwareMap);
        this.runtime = new ElapsedTime();
        this.intakeTimer = new ElapsedTime();
        this.waitTimer = new ElapsedTime();
        this.power = power;
        this.extendo_pos = extendoPositions;

        this.lastUpdateTimer = new ElapsedTime();

        addRequirements(claw, intake, arm);
    }

    @Override
    public void initialize() {
//        arm.forceDown();
        claw.open();
        pixelsDetectedState = 0;
        runtime.reset();
        intakeTimer.reset();
        intake.setPower(power);
        lastUpdateTimer.reset();
    }

//    @Override
//    public void execute() {
//        switch (pixelsDetectedState) {
//            case 0: // Pixel not detected
//                intake.setPower(power);
//                if (sensors.getBottomDistance() < DETECTION_THRESHOLD && sensors.getTopDistance() < DETECTION_THRESHOLD) {
//                    intakeTimer.reset();
//                    pixelsDetectedState = 1;
//                }
//                double deltaTime = lastUpdateTimer.milliseconds();
//                if (deltaTime >= 1000) { // Update position every second
//                    arm.setPosition(extendo_pos.getLeftPosition()-(POSITION_INCREMENT_PER_MILLISECOND * deltaTime));
//                }
//                break;
//            case 1: // Pixel detected, timer running
//                if (intakeTimer.milliseconds() < REQUIRED_TIME_MS) {
//
//                    intake.setPower(power);
//                    if (sensors.getBottomDistance() > DETECTION_THRESHOLD && sensors.getTopDistance() > DETECTION_THRESHOLD) {
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
//    }
//
//    @Override
//    public boolean isFinished() {
//        return pixelsDetectedState == 3;
//    }

//    public double getIntakeCurrent () {
//        return intake.getCurrent();
//    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            intake.setPower(0);
        }
        claw.close(); // Close the claw at the end of the command
//        arm.down();
    }
}