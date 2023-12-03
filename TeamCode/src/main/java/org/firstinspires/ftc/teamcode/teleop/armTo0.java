package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

@TeleOp
public class armTo0 extends LinearOpMode {
    private Arm arm;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(hardwareMap);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()){
            arm.setPosition(0);

            timer.reset();

            telemetry.update();
        }
    }
}