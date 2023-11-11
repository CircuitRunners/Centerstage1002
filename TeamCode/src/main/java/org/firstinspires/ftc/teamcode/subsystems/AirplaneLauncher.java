package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class AirplaneLauncher extends SubsystemBase {

    public enum ClawPosition {
        CLOSE(0.44),
        OPEN(0.54),
        FULL_OPEN(0.337); // nbot thing


        public double position;

        ClawPosition(double position){
            this.position = position;
        }
    }

    private ClawPosition clawPosition;

    private ServoImplEx claw;

    //Servos for the linkages
    public AirplaneLauncher(HardwareMap hardwareMap){
        //Retrieve servos from the hardware map
        claw = hardwareMap.get(ServoImplEx.class, "claw");

        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));

//        fullOpen();
        clawPosition = ClawPosition.OPEN;
    }

    public ClawPosition getClawPosition() {
        return clawPosition;
    }

    public void close() {
        claw.setPosition(ClawPosition.CLOSE.position);
        clawPosition = ClawPosition.CLOSE;
    }

    public void open() {
        claw.setPosition(ClawPosition.OPEN.position);
        clawPosition = ClawPosition.OPEN;
    }

    public void fullOpen() {
        claw.setPosition(ClawPosition.FULL_OPEN.position);
        clawPosition = ClawPosition.FULL_OPEN;
    }
}