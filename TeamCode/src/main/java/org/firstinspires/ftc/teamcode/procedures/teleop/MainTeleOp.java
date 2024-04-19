package org.firstinspires.ftc.teamcode.procedures.teleop;


import static org.firstinspires.ftc.teamcode.controllers.common.utilities.CrossBindings.circle;
import static org.firstinspires.ftc.teamcode.controllers.common.utilities.Utilities.debounce;
import static org.firstinspires.ftc.teamcode.controllers.common.utilities.Utilities.differential;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands.FollowPath;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.controllers.commands.intake.IntakeCommandEx;
import org.firstinspires.ftc.teamcode.controllers.commands.lift.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.controllers.commands.lift.ManualLiftResetCommand;
import org.firstinspires.ftc.teamcode.controllers.commands.presets.MoveToScoringCommandEx;
import org.firstinspires.ftc.teamcode.controllers.commands.presets.RetractOuttakeCommand;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Claw;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Intake;
import org.firstinspires.ftc.teamcode.procedures.TeleOpBase;

@Config
@TeleOp (name="Main TeleOp")
public class MainTeleOp extends TeleOpBase {
    public ManualLiftCommand manualLiftCommand;
    public IntakeCommandEx intakeCommand;
    public ManualLiftResetCommand manualLiftResetCommand;

    @Override
    public void onInitialize(){
        manualLiftCommand = new ManualLiftCommand(robot.lift, robot.arm, manipulator);
        manualLiftResetCommand = new ManualLiftResetCommand(robot.lift, manipulator);
        intakeCommand = new IntakeCommandEx(hardwareMap, robot.claw, robot.intake, robot.arm, Intake.IntakePowers.FAST);

        robot.lift.setDefaultCommand(new PerpetualCommand(manualLiftCommand));

        new Trigger(() -> manipulator.getLeftY() > 0.4)
                .whenActive(new MoveToScoringCommandEx(robot.lift, robot.arm, robot.claw, MoveToScoringCommandEx.Presets.MID, robot.pivot)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        new Trigger(() -> manipulator.getLeftY() < -0.4)
                .whenActive(new RetractOuttakeCommand(robot.lift, robot.arm, robot.claw, robot.pivot)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        //Mid preset
        new Trigger(() -> manipulator.getLeftX() > 0.6)
                .whenActive(new MoveToScoringCommandEx(robot.lift, robot.arm, robot.claw, MoveToScoringCommandEx.Presets.SHORT, robot.pivot)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        //Short preset
        new Trigger(() -> manipulator.getLeftX() < -0.6)
                .whenActive(new MoveToScoringCommandEx(robot.lift, robot.arm, robot.claw, MoveToScoringCommandEx.Presets.HIGH, robot.pivot)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        // pivoting
        new Trigger(() -> manipulator.getRightY() < -0.4)
                .whenActive(new InstantCommand(()-> {
                        if (!robot.lift.atLowerLimit()){
                            robot.pivot.center();
                        }
                }));
        new Trigger(() -> manipulator.getRightX() > 0.4)
                .whenActive(new InstantCommand(()-> {
                    if (robot.arm.getLeftPosition() > 0.5) {
                        if (!gamepad2.right_stick_button) {
                            robot.pivot.right();
                        } else {
                            // go 180
                            robot.pivot.rightEx();
                        }
                    }
                }));
        new Trigger(() -> manipulator.getRightX() < -0.4)
                .whenActive(new InstantCommand(()-> {
                    if (robot.arm.getLeftPosition() > 0.5) {
                        if (!gamepad2.right_stick_button) {
                            robot.pivot.left();
                        } else {
                            // go 180
                            robot.pivot.leftEx();
                        }
                    }
                }));

        manipulator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT) // Playstation Triangle
                .whenHeld(manualLiftResetCommand);

        driver.getGamepadButton(circle)
                .whenActive(intakeCommand);

        robot.claw.open();
        robot.lift.initialInitHang();
        robot.lift.resetLiftPosition();

        telemetry.addLine("Ready for start!");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        drivebase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.cross);

        telemetry.addData("Current", intakeCommand.getIntakeCurrent());

         // IN BETA TEST THISIOWHADHSOD

        // Reset heading with MATH
        if (gamepad1.square) {
            drivebase.resetHeading();
            gamepad1.rumble(50);
        }


//        if (gamepad2.triangle) {
//            robot.lift.setLiftPower(-0.2);
//            robot.lift.hangPower(-0.0000002);
//        } else
        robot.lift.hangPower(0);
        if (gamepad2.cross) {
            robot.lift.hangPower(-1.0);
            robot.lift.setLiftPower(0.0);
        } else {
            robot.lift.hangPower(0);
        }

        if (gamepad2.dpad_right) {
            robot.lift.hangPower(0.2);
        }

        // Intake Assembly
        if (debounce(gamepad1.left_trigger) || debounce(gamepad1.right_trigger)) {
            if (intakeCommand.isScheduled()) {
                telemetry.addLine("ICSX2");
                intakeCommand.cancel();
            }
            robot.intake.setPower(gamepad1.right_trigger, gamepad1.left_trigger);
        } else if (intakeCommand.isScheduled()){
            telemetry.addLine("Intake Command Scheduled");
        } else {
            robot.intake.setPower(0);
        }

        // Airplane Launcher
        robot.airplaneLauncher.processInput(gamepad1.dpad_down, false);

        // Transfer/Claw
        if (gamepad2.right_bumper) {
            robot.claw.close();
        } else if (gamepad2.left_bumper) {
            robot.claw.open();
        }
        if (gamepad2.left_bumper) {
            if (differential(robot.claw.getPosition() - Claw.IntakePositions.OPEN.position, 0.001)) {
                robot.claw.close();
            } else {
                robot.claw.open();
            }
        }

        // Arm commands
        if (debounce(gamepad2.right_trigger)) { // outtake
            robot.pivot.center();
            robot.arm.up();
        } else if (debounce(gamepad2.left_trigger)) { //intake
            robot.pivot.center();
            robot.arm.down();
        }

        // Front "Extendo" Arm up/down
        if (gamepad1.left_bumper){
            robot.frontArm.up();
        } else if(gamepad1.right_bumper){
            robot.frontArm.down();
        }

        if (gamepad2.circle){
            robot.lift.enableHang();
        } else if (gamepad2.square) {
            robot.lift.initialInitHang();
        }

        //telemetry.addData("Distance Bottom", intakeCommand.sensors.getBottomDistance());
        //telemetry.addData("Distance Top", intakeCommand.sensors.getTopDistance());
        telemetry.addData("Claw Status", (robot.claw.getPosition() < 0.45) ? "Open": "Closed");
        telemetry.addData("Arm Position", robot.arm.getLeftPosition());
        telemetry.addData("Pivot Position", robot.pivot.getPosition());

//        if (
//                intakeCommand.sensors.getTopDistance() < 5 && intakeCommand.sensors.getTopDistance() > 1
//                && intakeCommand.sensors.getBottomDistance() < 5 && intakeCommand.sensors.getBottomDistance() > 1
//                && robot.claw.getPosition() < 0.45
//                && robot.arm.getLeftPosition() < 0.3) {
//            gamepad2.rumble(200);
//            gamepad1.rumble(200);
//        }

        // Telemetry for Path Testing
//        drive.update();
        telemetry.addData("IMU", robot.navx_device.getYaw());
        telemetry.addData("imu2", drivebase.getCorrectedYaw());


        // Ensure telemetry actually works
        telemetry.update();
    }

}