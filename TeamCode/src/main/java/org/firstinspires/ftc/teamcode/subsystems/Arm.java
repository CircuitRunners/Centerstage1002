package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm extends SubsystemBase {

    public enum ArmPositions {
        DOWN(0.03),
        SCORING(0.64), //.669
        GROUND(0.7);

        public double position;

        ArmPositions(double position){
            this.position = position;
        }
    }

    // right arm .448 up
    // right .9045 down
    // .54 claw open
    // .44 claw closed
    // left arm up .488
    // left arm .9499

//    public enum LeftArmPositions {
//        DOWN(0.9499),
//        UP (0.488)
//    }


    private ServoImplEx leftServo;
    private ServoImplEx rightServo;


    //motion profile constraints
    private TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(1.6, 1.8);

    //Motion profile, initialize to down
    private TrapezoidProfile armProfile =
            new TrapezoidProfile(constraints, new TrapezoidProfile.State(ArmPositions.DOWN.position, 0),
                    new TrapezoidProfile.State(ArmPositions.DOWN.position, 0)
            );

    private ElapsedTime timer = new ElapsedTime();
    private double prevTarget = ArmPositions.DOWN.position;


    private double clawFullOpenLimit = 0.3;


    public Arm(HardwareMap hardwareMap){

        leftServo = hardwareMap.get(ServoImplEx.class, "leftArm");
        rightServo = hardwareMap.get(ServoImplEx.class, "rightArm");

        //Set the maximum pwm range
        leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        forceDown();
    }

    @Override
    public void periodic(){


        if(!armProfile.isFinished(timer.seconds())) {
            //Read the current target for the profile
            double position = armProfile.calculate(timer.seconds()).position;

            //Set servo positions
            leftServo.setPosition(position);
            rightServo.setPosition(position);

        }

    }

    //Set the servos to a numerical position
    public void setPosition(double target) {

        //Create a new profile starting from the last position command
        if(prevTarget != target){
            armProfile = new TrapezoidProfile(
                    constraints,
                    new TrapezoidProfile.State(target, 0),
                    new TrapezoidProfile.State(leftServo.getPosition(), 0)
            );


            //Reset the timer
            timer.reset();
        }

        prevTarget = target;

    }

    public double getPosition(){
        return leftServo.getPosition();
    }

    //All the way to the rest position
    public void down(){
        setPosition(ArmPositions.DOWN.position);
    }

    //Bypasses the profile
    public void forceDown(){
        leftServo.setPosition(ArmPositions.DOWN.position);
        rightServo.setPosition(ArmPositions.DOWN.position);
    }


    //Set a preset level
    public void setLevel(ArmPositions level){
        setPosition(level.position);
    }


}