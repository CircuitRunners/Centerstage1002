package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.teamcode.utilities.Utilities.debounce;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftResetCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoArm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utilities.BulkCacheCommand;


@TeleOp (name="Idler (powersave)")
public class Idler extends CommandOpMode {
    private ServoImplEx gripServo, leftServo, rightServo, rightArmServo, leftArmServo;
    private ServoImplEx[] servos;


    private AHRS navx_device;

    @Override
    public void initialize(){
        // Use a bulk cache to loop faster using old values instead of blocking a thread kinda
        schedule(new BulkCacheCommand(hardwareMap));

        // Subsystems
        gripServo = hardwareMap.get(ServoImplEx.class, "claw");
        leftServo = hardwareMap.get(ServoImplEx.class, "leftArm");
        rightServo = hardwareMap.get(ServoImplEx.class, "rightArm");
        leftArmServo = hardwareMap.get(ServoImplEx.class, "leftArmServo");
        rightArmServo = hardwareMap.get(ServoImplEx.class, "rightArmServo");

        servos = new ServoImplEx[]{gripServo,leftServo, rightServo, leftArmServo, rightArmServo};

        for (ServoImplEx servo : servos) {
            servo.setPwmDisable();
        }
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        super.run();
    }
}