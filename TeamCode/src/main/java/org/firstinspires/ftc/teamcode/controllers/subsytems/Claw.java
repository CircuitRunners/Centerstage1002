package org.firstinspires.ftc.teamcode.controllers.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Claw extends SubsystemBase {
    private ServoImplEx gripServo;

    public Claw(HardwareMap hardwareMap) {
        gripServo = hardwareMap.get(ServoImplEx.class, "claw");
    }

    public enum IntakePositions {
        OPEN(0.39),
        CLOSE(0.56);

        public double position;

        IntakePositions(double position){
            this.position = position;
        }
    }

    public void close(){
        setPosition(IntakePositions.CLOSE.position);
    }

    public void open(){
        setPosition(IntakePositions.OPEN.position);
    }

    public void setPosition(double position) {
        gripServo.setPosition(position);
    }

    public double getPosition() {
        return gripServo.getPosition();
    }

    public void tuningModeOn() {
        gripServo.setPwmDisable();
    }

    public void tuningModeOff() {
        gripServo.setPwmEnable();
    }
}