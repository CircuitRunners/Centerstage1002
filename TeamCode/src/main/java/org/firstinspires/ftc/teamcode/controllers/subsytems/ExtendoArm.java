package org.firstinspires.ftc.teamcode.controllers.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class ExtendoArm extends SubsystemBase {
    private ServoImplEx leftServo;
    private ServoImplEx rightServo;

    public enum ExtendoPositions {
        DOWN(0.30, 0.60),
        TRANSPORT(0.67,0.34),
        TRANSPORT_ALPHA(0.649,0.359),
        UP(0.76,0.29),
        PIXEL1(0.35,0.56),
        PIXEL2(0.35,0.58),
        PIXEL3(0.35,0.60),
        PIXEL4(0.35,0.63),
        PIXEL5(0.35,0.66);

        private final double position_right;
        private final double position_left;

        ExtendoPositions(double position_left, double position_right) {
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

    public void setPosition(ExtendoPositions target) {
        // Create a new profile starting from the last position command
        leftServo.setPosition(target.getLeftPosition());
        rightServo.setPosition(target.getRightPosition());
    }


    public void setPosition(double target_left, double target_right) {
        // Create a new profile starting from the last position command
        leftServo.setPosition(target_left);
        rightServo.setPosition(target_right);
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
        setPosition(ExtendoPositions.DOWN);
    }

    public void up() {
        setPosition(ExtendoPositions.UP);
    }
    public void mid() {
        setPosition(ExtendoPositions.TRANSPORT);
    }
    public void alpha () {
        setPosition(ExtendoPositions.TRANSPORT_ALPHA);
    }

    public void toPixel1 () {
        setPosition(ExtendoPositions.PIXEL1);
    }
    public void toPixel2 () {
        setPosition(ExtendoPositions.PIXEL2);
    }
    public void toPixel3 () {
        setPosition(ExtendoPositions.PIXEL3);
    }
    public void toPixel4 () {
        setPosition(ExtendoPositions.PIXEL4);
    }
    public void toPixel5 () {
        setPosition(ExtendoPositions.PIXEL5);
    }
}