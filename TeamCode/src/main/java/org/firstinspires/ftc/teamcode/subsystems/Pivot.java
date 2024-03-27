package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Pivot.PivotPositions.INTAKE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

//turn right, gamepad2 left joystick right
//turn left, gamepad2 left joystick left
//reset (pivot, lift, arm), gamepad2 left joystick up
//pivot preset, gamepad2 left joystick down

public class Pivot extends SubsystemBase{

    private ServoImplEx pivotServo;
    private String servoHardwareMapName = "rotation"; // sussy baka richard behavior for rotationServo

    public enum PivotPositions{
        INTAKE(0);

        public final double position;

        PivotPositions(double position) {
            this.position = position;
        }
    }

    public Pivot(HardwareMap hardwareMap){
        pivotServo = hardwareMap.get(ServoImplEx.class, servoHardwareMapName);
    }



    public void reset(){
        pivotServo.setPosition(INTAKE.position);
    }


}
