package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Photon
public class IntakeCommand extends CommandBase {
    private ElapsedTime runtime;

    private Intake intake;
    private DistanceSensor distanceSensor;
    private int pixelsDetectedState = 0;

    private boolean isClawClosed;

    private Claw claw;

    public IntakeCommand(HardwareMap hardwareMap, Claw claw, Intake intake){
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        this.claw = claw;
        this.intake = intake;
        runtime = new ElapsedTime();
    }


    @Override
    public void initialize() {
        claw.open();
        //once
    }

    //Run repeatedly while the command is active
    @Override
    public void execute(){
        if (distanceSensor.getDistance(DistanceUnit.CM) > 4) {
            intake.setPower(Intake.IntakePowers.FAST);
        } else {
            if (pixelsDetectedState == 0) { // not detected
                pixelsDetectedState = 1;
                runtime.reset();
            }
        }
        if (pixelsDetectedState == 1) {
            double ms = runtime.milliseconds();
            if (ms < 500) {
                intake.setPower(Intake.IntakePowers.NORMAL);
            }
            else if (ms < 2000) {
                if (!isClawClosed) {
                    claw.close();
                }
            }
            else if (ms < 2500) {
                intake.setPower(0);
            }
            else if (ms < 5000) {
                intake.setPower(Intake.OuttakePowers.FAST);
            }
            else {
                pixelsDetectedState = 2;
            }
        }
        if (pixelsDetectedState == 2) {
            intake.setPower(0);
        }
        //Update the lift power with the controller
    }

    @Override
    public boolean isFinished(){
        //End if the lift position is within the tolerance
        return pixelsDetectedState == 2;
    }

}