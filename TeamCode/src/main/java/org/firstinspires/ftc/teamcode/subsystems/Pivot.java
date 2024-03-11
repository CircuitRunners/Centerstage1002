package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

//turn right, gamepad2 left joystick right
//turn left, gamepad2 left joystick left
//reset (pivot, lift, arm), gamepad2 left joystick up
//pivot preset, gamepad2 left joystick down

public class Pivot extends SubsystemBase{

    private ServoImplEx pivotServo;
    private String servoHardwareMapName = "pivot";

    public enum PivotPositions{
        INTAKE(0);

        public final double position;

    }

    public Pivot(HardwareMap hardwareMap){
        pivotServo = hardwareMap.get(ServoImplEx.class, servoHardwareMapName);
    }



    public void reset(){
        pivotServo.setPosition(INTAKE);
    }


}
