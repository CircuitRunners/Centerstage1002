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
    private double hangingMotorPower;
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
    private double hangingUpInitialPosition;
    private double hangingUpExtendedPosition;
    private double hangingUpPosition = hangingUpInitialPosition;
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    DcMotorEx leftLiftMotor, rightLiftMotor;
    DcMotorEx intakeMotor;
    Servo airplaneServo;
    Servo leftClawServo, rightClawServo;
    Servo leftArmServo, rightArmServo;
    Servo hangingServo;
    DcMotorEx hangingMotor;
    Servo angleServo;


    @Override
    public void initialize() {
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLift");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftWheel");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftWheel");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightWheel");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightWheel");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        airplaneServo = hardwareMap.get(Servo.class, "airplane");
        leftClawServo = hardwareMap.get(Servo.class, "leftClaw");
        rightClawServo = hardwareMap.get(Servo.class, "rightClaw");
        leftArmServo = hardwareMap.get(Servo.class, "leftArm");
        rightArmServo = hardwareMap.get(Servo.class, "rightArm");
        hangingServo = hardwareMap.get(Servo.class, "hangingUp");
        hangingMotor = hardwareMap.get(DcMotorEx.class, "hangingDown");
        angleServo = harwareMap.get(Servo.class, "angle");


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
        double hangingDown = gamepad1.b;
        double angle = gamepad2.right_stick_y;

        boolean intake = gamepad1.right_bumper;
        boolean reverseIntake = gamepad1.left_bumper;
        boolean airPlaneLaunch = gamepad2.y;
        boolean leftClawToggle = gamepad2.x;
        boolean rightClawToggle = gamepad2.b;
        boolean bothClawToggle = gamepad2.a;
        boolean armToggle = gamepad2.right_bumper;
        boolean hangingUpToggle = gamepad1.a;


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
        if(hangingUpToggle){
            if(hangingUpPosition == hangingUpExtendedPosition){
                hangingUpPosition = hangingUpInitialPosition;
            }
            if(hangingUpPosition == hangingUpInitialPosition){
                hangingUpPosition = hangingUpExtendedPosition;
            }
        }
        frontLeftPower = Range.clip(-(y-x), -1.0, 1.0);
        backLeftPower = Range.clip(-(y-x), -1.0, 1.0);
        frontRightPower = Range.clip(y+x, -1.0, 1.0);
        backRightPower = Range.clip(y+x, -1.0, 1.0);
        hangingMotorPower = Range.clip(y+x, -1.0, 1.0);
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
        leftArmServo.setPosition(leftArmPosition);
        rightArmServo.setPosition(rightArmPosition);

    }

}
