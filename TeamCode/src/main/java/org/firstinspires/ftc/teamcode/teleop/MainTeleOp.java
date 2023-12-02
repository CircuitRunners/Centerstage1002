package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.teamcode.utilities.Utilities.debounce;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftResetCommand;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.utilities.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.Lift;



@TeleOp (name="MainTeleOp")
public class MainTeleOp extends CommandOpMode {
    private DcMotorEx frontLeft, backLeft, frontRight, backRight, intakeMotor;
    private DcMotorEx[] drivebaseMotors;
    private double frontLeftPower, backLeftPower, frontRightPower, backRightPower;

    private Lift lift;
    private AirplaneLauncher airplaneLauncher;
    private Drivebase drivebase;
    private Arm arm;
    private Transfer transfer;
    private Intake intake;

    private ServoImplEx rightArm, leftArm, claw;
    private IMU imu;

    private ManualLiftCommand manualLiftCommand;
    private ManualLiftResetCommand manualLiftResetCommand;

    private boolean isUp = false, isDown = true, isTransport = false, isPressed = false;
    private double upOffset, downOffset, transportOffset;
    private double overallOffset = 0.05;

    @Override
    public void initialize(){
        // Use a bulk cache to loop faster using old values instead of blocking a thread kinda
        schedule(new BulkCacheCommand(hardwareMap));

        // Subsystems
        lift = new Lift(hardwareMap);
        airplaneLauncher = new AirplaneLauncher(hardwareMap);
        drivebase = new Drivebase(hardwareMap);
        arm = new Arm(hardwareMap);
        transfer = new Transfer(hardwareMap);
        intake = new Intake(hardwareMap);
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        super.run();

        drivebase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Reset heading with MATH
        if (gamepad1.square) {
            drivebase.resetHeading();
            gamepad1.rumble(50);
        }


        // Lift brakes when not doing anything
        lift.brake();

        if (debounce(gamepad2.left_stick_y)) {
            lift.setLiftPower(gamepad2.left_stick_y);
        }

        // Intake Assembly
        intake.setPower(gamepad1.left_trigger, gamepad1.right_trigger);

        // Airplane Launcher
        airplaneLauncher.processInput(gamepad2.cross, false);

        // Transfer/Claw
        if (gamepad1.right_bumper) {
            transfer.close();
        } else if (gamepad1.left_bumper) {
            transfer.open();
        }

        // Arm commands
        if (gamepad2.right_bumper) { // outtake
            arm.toPosition(Arm.ArmPositions.SCORING.position + upOffset);

            isDown = false;
            isUp = true;
            isTransport = false;
        } else if (gamepad2.left_bumper) { //intake
            arm.toPosition(Arm.ArmPositions.DOWN.position + downOffset);

            isDown = true;
            isUp = false;
            isTransport = false;
        } else if (gamepad2.square) { //transport
            arm.toPosition(Arm.ArmPositions.TRANSPORT.position + transportOffset);

            isDown = false;
            isUp = false;
            isTransport = true;
        }

        int sign = 0;
        if (gamepad2.dpad_up) {
            sign = 1;
        } else if (gamepad2.dpad_down) {
            sign = -1;
        } else {
            isPressed = false;
        }

        if (sign != 0 && !isPressed) {
            isPressed = true;
            if (isUp) {
                upOffset += sign*overallOffset;
                arm.toPosition(Arm.ArmPositions.SCORING.position + upOffset);
            } else if (isDown) {
                downOffset += sign*overallOffset;
                arm.toPosition(Arm.ArmPositions.DOWN.position + downOffset);
            } else if (isTransport) {
                transportOffset += sign*overallOffset;
                arm.toPosition(Arm.ArmPositions.TRANSPORT.position + transportOffset);
            }
        }

        telemetry.addData("Offsets", String.format("up %f, down %f, trans %f", upOffset, downOffset, transportOffset));

        telemetry.update();
    }
}