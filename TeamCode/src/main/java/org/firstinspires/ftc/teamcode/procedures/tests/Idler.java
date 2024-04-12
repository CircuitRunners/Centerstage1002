package org.firstinspires.ftc.teamcode.procedures.tests;


import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.controllers.common.utilities.BulkCacheCommand;

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