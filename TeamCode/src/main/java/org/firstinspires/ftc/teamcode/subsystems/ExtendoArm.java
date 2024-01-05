package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExtendoArm extends SubsystemBase {
    private ServoImplEx leftServo;
    private ServoImplEx rightServo;

    public enum ArmPositions {
        DOWN(0.72, 0.3),
        TRANSPORT(0.0,0.5),
        UP(0.4,0.65);

        private final double position_right;
        private final double position_left;

        ArmPositions(double position_left, double position_right) {
            this.position_right = position_right;
            this.position_left = position_left;
        }

        public double getRightPosition() {
            return this.position_right;
        }

        public double getLeftPosition() {
            return this.position_left;
        }
    }

    public ExtendoArm(HardwareMap hardwareMap){
        leftServo = hardwareMap.get(ServoImplEx.class, "leftArmServo");
        rightServo = hardwareMap.get(ServoImplEx.class, "rightArmServo");

        up();
    }

    public void setPosition(ArmPositions target) {
        // Create a new profile starting from the last position command
        leftServo.setPosition(target.getLeftPosition());
        rightServo.setPosition(target.getRightPosition());
    }

    // Set the servos to a numerical position -- love polymorphism <3
    public void setPosition(double target) {
        // Create a new profile starting from the last position command
        leftServo.setPosition(target);
        rightServo.setPosition(1.0-target);
    }

//    SETTERS

    // All the way to the rest position
    public void down(){
        setPosition(ArmPositions.DOWN);
    }

    public void up(){
        setPosition(ArmPositions.UP);
    }
}