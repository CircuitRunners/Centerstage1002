package org.firstinspires.ftc.teamcode.procedures.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controllers.subsytems.Lift;

@Disabled
@TeleOp
public class LiftPositionOutput extends LinearOpMode {
    private Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {

        lift = new Lift(hardwareMap);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()){
            double position = lift.getLiftPosition();
            double velocity = lift.getLiftVelocity();
            double acceleration = velocity / timer.seconds();

//            lift.setLiftPower(0.5);

            timer.reset();

            telemetry.addData("Lift Position", position );
            telemetry.addData("Lift Velocity", velocity );
            telemetry.addData("Lift Acceleration", acceleration );
            telemetry.update();
        }
    }
}