package org.firstinspires.ftc.teamcode.procedures;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.controllers.RobotCore;
import org.firstinspires.ftc.teamcode.controllers.subsytems.Drivebase;

public abstract class TeleOpBase extends CommandOpMode {
    public RobotCore robot;
    public GamepadEx driver;
    public GamepadEx manipulator;

    public Drivebase drivebase;

    @Override
    public void initialize() {
        robot = new RobotCore(hardwareMap);

        drivebase = new Drivebase(hardwareMap);
        driver = new GamepadEx(gamepad1);
        manipulator = new GamepadEx(gamepad2);

        onInitialize();
    }

    public abstract void onInitialize ();
}
