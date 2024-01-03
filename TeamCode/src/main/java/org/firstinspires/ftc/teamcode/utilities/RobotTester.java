package org.firstinspires.ftc.teamcode.utilities;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@TeleOp(name = "RobotTester", group = "Command")
public class RobotTester extends CommandOpMode {

    // Declare subsystems
    private AirplaneLauncher airplaneLauncher;
    private Arm arm;
    private Intake intake;
    private Lift lift;
    private Transfer transfer;
    private boolean testing = false;
    private double stepValue = 0.1;
    private double intakePower = 0;

    @Override
    public void initialize() {
        // Initialize subsystems with hardwareMap
        airplaneLauncher = new AirplaneLauncher(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        transfer = new Transfer(hardwareMap);

        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);

        // Define buttons for tuning mode toggles
        GamepadButton enableTuningButton = new GamepadButton(driver, GamepadKeys.Button.X);
        GamepadButton disableTuningButton = new GamepadButton(driver, GamepadKeys.Button.Y);

        enableTuningButton.whenPressed(() -> {
            testing = true;
            airplaneLauncher.tuningModeOn();
            arm.tuningModeOn();
            transfer.tuningModeOn();
        });

        disableTuningButton.whenPressed(() -> {
            testing = false;
            airplaneLauncher.tuningModeOff();
            arm.tuningModeOff();
            transfer.tuningModeOff();
        });

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                (()->{ stepValue /= 10; })
        );
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                (()->{ stepValue *= 10; })
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                (()->{ if (intakePower <= 1 - stepValue) intakePower += stepValue; })
        );
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                (()->{ if (intakePower >= 0 + stepValue) intakePower -= stepValue; })
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                (()->{ if (intakePower <= 1 - (10 * stepValue)) intakePower += 10*stepValue; })
        );
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                (()->{ if (intakePower >= 0 + 10*stepValue) intakePower -= 10*stepValue; })
        );
    }

    @Override
    public void run() {
        super.run();
        if (testing) {
            if ((Math.abs(gamepad1.left_stick_y) > 0.01) && gamepad1.square) intakePower = gamepad1.left_stick_y;
        }
        telemetry.addData("Arm", arm.getState());
        telemetry.addData("Airplane Launcher", airplaneLauncher.getPosition());
        telemetry.addData("Transfer", transfer.getPosition());

        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Step Value", stepValue);
        telemetry.addData("Step Value", gamepad1.left_stick_y);

        telemetry.addData("Lift Position", lift.getLiftPosition());
        intake.setPower(intakePower);

        telemetry.update();
    }
}