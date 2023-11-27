package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Transfer extends SubsystemBase {
    private ServoImplEx gripServo;

    public Transfer (HardwareMap hardwareMap) {
        gripServo = hardwareMap.get(ServoImplEx.class, "claw");
    }

    public enum IntakePositions {
        FULLOPEN(0.03),
        OPEN(0.03),
        CLOSE(0.7);

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

    public void fullopen(){
        setPosition(IntakePositions.FULLOPEN.position);
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