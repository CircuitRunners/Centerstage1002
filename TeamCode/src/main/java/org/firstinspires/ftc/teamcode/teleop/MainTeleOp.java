package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MainTeleOp extends CommandOpMode {

    private double liftPower;
    private double intakePower;
    private double frontLeftPower;
    private double backLeftPower;
    private double frontRightPower;
    private double backRightPower;
    private double initialAirplanePosition;
    private double airplanePosition = initialAirplanePosition;
    private double leftClawPosition = 0.0;
    private double rightClawPosition = 0.0;
    private double leftArmPosition = 0.0;
    private double rightArmPosition = 0.0;
    private double extendedArmPosition;
    private double initialArmPosition;
    private double initialClawPosition;
    private double extendedClawPosition;
    private double finalAirplanePosition;
    private double intakePowerValue = 1;
    private double outtakePowerValue = -1;
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    DcMotorEx leftLiftMotor, rightLiftMotor;
    DcMotorEx intakeMotor;
    Servo airplaneServo;
    Servo leftClawServo;
    Servo rightClawServo;
    Servo leftArm, rightArm;


    @Override
    public void initialize() {
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLift");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftWheel");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftWheel");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightWheel");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightWheel");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        airplaneServo = hardwareMap.get(Servo.class, "Airplane");
        leftClawServo = hardwareMap.get(Servo.class, "Left Claw");
        rightClawServo = hardwareMap.get(Servo.class, "Right Claw");
        leftArm = hardwareMap.get(Servo.class, "Left arm servo");
        rightArm = hardwareMap.get(Servo.class, "Right arm servo");


    }
    @Override
    public void run() {
        super.run();
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double liftUp = gamepad2.left_trigger;
        double liftDown = gamepad2.right_trigger;
        double rx = gamepad1.right_stick_x;
        double x2 = gamepad2.left_stick_x;

        boolean intake = gamepad1.right_bumper;
        boolean reverseIntake = gamepad1.left_bumper;
        boolean airPlaneLaunch = gamepad2.y;
        boolean leftClawToggle = gamepad2.x;
        boolean rightClawToggle = gamepad2.b;
        boolean bothClawToggle = gamepad2.a;
        boolean armToggle = gamepad2.right_bumper;


        if (liftUp != 0) {
            liftPower = liftUp;
        }
        if (liftDown != 0) {
            liftPower = liftDown;
        }

        if (intake){
            intakePower = intakePowerValue;
        }
        else if (reverseIntake){
            intakePower = outtakePowerValue;
        }
        else{
            intakePower = 0;
        }

        if(airPlaneLaunch){
            airplanePosition = finalAirplanePosition;
        }
        if(leftClawToggle){
            if(leftClawPosition == initialClawPosition){
                leftClawPosition = extendedClawPosition;
            }
            else if(leftClawPosition == extendedClawPosition){
                leftClawPosition = initialClawPosition;
            }

        }
        if(rightClawToggle){
            if(rightClawPosition == initialClawPosition){
                rightClawPosition = extendedClawPosition;
            }
            else if(rightClawPosition == extendedClawPosition){
                rightClawPosition = initialClawPosition;
            }
        }
        if(bothClawToggle){
            if(leftClawPosition == initialClawPosition && rightClawPosition == initialClawPosition){
                leftClawPosition = extendedClawPosition;
                rightClawPosition = extendedClawPosition;
            }
            else if (leftClawPosition == extendedClawPosition && rightClawPosition == extendedClawPosition){
                leftClawPosition = initialClawPosition;
                rightClawPosition = initialClawPosition;
            }
        }
        if(armToggle){
            if(leftArmPosition == initialArmPosition && rightArmPosition == initialArmPosition){
                leftArmPosition = extendedArmPosition;
                rightArmPosition = extendedArmPosition;
            }
            if(leftArmPosition == extendedArmPosition && rightArmPosition == extendedArmPosition){
                leftArmPosition = initialArmPosition;
                rightArmPosition = initialArmPosition;
            }
        }
        frontLeftPower = Range.clip(-(y-x), -1.0, 1.0);
        backLeftPower = Range.clip(-(y-x), -1.0, 1.0);
        frontRightPower = Range.clip(y+x, -1.0, 1.0);
        backRightPower = Range.clip(y+x, -1.0, 1.0);
        leftLiftMotor.setPower(liftPower);
        rightLiftMotor.setPower(-liftPower);
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        intakeMotor.setPower(intakePower);
        airplaneServo.setPosition(airplanePosition);
        leftClawServo.setPosition(leftClawPosition);
        rightClawServo.setPosition(rightClawPosition);

    }

}
