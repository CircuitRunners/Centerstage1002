package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class AntiHero extends CommandOpMode {

    private double power = -1;
    private DcMotorEx
    DcMotorEx leftLiftMotor, rightLiftMotor;

    @Override
    public void initialize() {
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLift");
    }
    @Override
    public void run() {
        super.run();

        leftLiftMotor.setPower(power);
        rightLiftMotor.setPower(-power);
    }

}
