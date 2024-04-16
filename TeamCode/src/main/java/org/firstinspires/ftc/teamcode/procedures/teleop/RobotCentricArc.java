package org.firstinspires.ftc.teamcode.procedures.teleop;


import static org.firstinspires.ftc.teamcode.controllers.common.utilities.CrossBindings.circle;
import static org.firstinspires.ftc.teamcode.controllers.common.utilities.Utilities.debounce;
import static org.firstinspires.ftc.teamcode.controllers.common.utilities.Utilities.differential;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.commands.intake.IntakeCommandEx;
import org.firstinspires.ftc.teamcode.controllers.commands.lift.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.controllers.commands.lift.ManualLiftResetCommand;
import org.firstinspires.ftc.teamcode.controllers.commands.presets.MoveToScoringCommandEx;
import org.firstinspires.ftc.teamcode.controllers.commands.presets.RetractOuttakeCommand;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Claw;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Intake;
import org.firstinspires.ftc.teamcode.procedures.TeleOpBase;

@Config
@TeleOp (name="Lite Robot TeleOp")
public class RobotCentricArc extends TeleOpBase {

    @Override
    public void onInitialize(){
        telemetry.addLine("Ready for start!");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;


        double leftFrontMotorPower = y + x + rx;
        double rightFrontMotorPower = y - x - rx;
        double leftBackMotorPower = y - x + rx;
        double rightBackMotorPower = y + x - rx;

        drivebase.driveRobotPowers(leftFrontMotorPower, leftBackMotorPower, rightFrontMotorPower, rightBackMotorPower);

        // Ensure telemetry actually works
        telemetry.update();
    }
}