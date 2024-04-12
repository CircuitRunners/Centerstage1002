package org.firstinspires.ftc.teamcode.procedures.tests;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.controllers.common.util.Encoder;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.BulkCacheCommand;

@TeleOp (name="Testers De Servoiotus")
public class ServoTesters extends CommandOpMode {

    private Encoder leftEncoder, rightEncoder, frontEncoder;
    private double normleft, normright, normfront;

    @Override
    public void initialize(){
        // Use a bulk cache to loop faster using old values instead of blocking a thread kinda
        schedule(new BulkCacheCommand(hardwareMap));
//        PhotonCore.start(hardwareMap);

        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);



        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "winch"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightLift"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backRight"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        normleft = leftEncoder.getCurrentPosition();
        normright = rightEncoder.getCurrentPosition();
        normfront = frontEncoder.getCurrentPosition();


        telemetry.addLine("Ready for start!");
        telemetry.update();
    }


    @Override
    public void run() {
        super.run();

        telemetry.addData("norm L", normleft - leftEncoder.getCurrentPosition());
        telemetry.addData("norm R", normright - rightEncoder.getCurrentPosition());
        telemetry.addData("norm Front/Perp", normfront - frontEncoder.getCurrentPosition());

        telemetry.addData("leftEncoder", leftEncoder.getCurrentPosition());
        telemetry.addData("rightEncoder", rightEncoder.getCurrentPosition());
        telemetry.addData("frontEncoder", frontEncoder.getCurrentPosition());

        // Ensure telemetry actually works
        telemetry.update();
    }
}