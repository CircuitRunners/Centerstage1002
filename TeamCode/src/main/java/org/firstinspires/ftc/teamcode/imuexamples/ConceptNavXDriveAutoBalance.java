/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
package org.firstinspires.ftc.teamcode.imuexamples;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

/*
 * An example loop op mode where the robot will balance automatically,
 * based upon the navX-Micro pitch/roll angles.
 *
 * The basic principle used in the example is based upon measurement
 * of the navX-Micro Pitch (rotation about the X axis) and Roll
 * (rotation about the Y axis) angles. When these angles exceed the
 * “off balance” threshold and until these angles fall below the
 * “on balance” threshold, the drive system is automatically driven in
 * the opposite direction at a magnitude proportional to the Pitch or Roll
 * angle.
 *
 * Note that this is just a starting point for automatic balancing, and will
 * likely require a reasonable amount of tuning in order to work well with
 * your robot.  The selection of the magnitude of correction to apply to
 * the drive motors in response to pitch/roll angle changes could be replaced
 * by a PID controller in order to provide a tuning mechanism appropriate to the robot.
 */

@TeleOp(name = "Concept: navX Auto-Balance - Loop", group = "Concept")
// @Disabled Comment this in to remove this from the Driver Station OpMode List
public class ConceptNavXDriveAutoBalance extends OpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;

    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    static final double kOffBalanceAngleThresholdDegrees = 5.0f;
    static final double kOnBalanceAngleThresholdDegrees  = 5.0f;

    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;

    private boolean calibration_complete = false;
    private boolean autoBalanceXMode = false;
    private boolean autoBalanceYMode = false;

    DecimalFormat df;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");

        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        /* If possible, use encoders when driving, as it results in more */
        /* predictable drive system response.                           */
        //leftMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //rightMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        df = new DecimalFormat("#.##");
    }

    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if ( !calibration_complete ) {
            /* NOTE:  If your robot uses the navX-Micro yaw angle, ensure that calibration
               completes successfully.  However if your robot only uses the pitch
               and roll angles (this example only uses pitch and roll angles) then
               it is not required to allow the sensor to complete calibration before
               using it.
             */
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if ( calibration_complete ) {
                navx_device.zeroYaw();
            } else {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        } else {
            double xAxisRate = gamepad1.left_stick_x;
            // (note: The joystick goes negative when pushed forwards, so negate it)
            double yAxisRate = -gamepad1.left_stick_y;
            double pitchAngleDegrees = navx_device.getPitch();
            double rollAngleDegrees = navx_device.getRoll();

            if (!autoBalanceXMode &&
                    (Math.abs(pitchAngleDegrees) >=
                            Math.abs(kOffBalanceAngleThresholdDegrees))) {
                autoBalanceXMode = true;
            } else if (autoBalanceXMode &&
                    (Math.abs(pitchAngleDegrees) <=
                            Math.abs(kOnBalanceAngleThresholdDegrees))) {
                autoBalanceXMode = false;
            }
            if (!autoBalanceYMode &&
                    (Math.abs(pitchAngleDegrees) >=
                            Math.abs(kOffBalanceAngleThresholdDegrees))) {
                autoBalanceYMode = true;
            } else if (autoBalanceYMode &&
                    (Math.abs(pitchAngleDegrees) <=
                            Math.abs(kOnBalanceAngleThresholdDegrees))) {
                autoBalanceYMode = false;
            }

            // Control drive system automatically,
            // driving in reverse direction of pitch/roll angle,
            // with a magnitude based upon the angle

            if ( autoBalanceXMode ) {
                double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
                xAxisRate = Math.sin(pitchAngleRadians) * -1;
            }

            if ( autoBalanceYMode ) {
                double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
                yAxisRate = Math.sin(rollAngleRadians) * -1;
            }

            // At this point, the X/Axis motion rates are proportional to the
            // angle, and in the inverse direction.

            // NOTE:  This algorithm assumes an omni-directional drive system (e.g., Mecanum)
            // that can navigate linearly in both X and Y axis direction.  Tank-style drive
            // systems (without the ability to travel in a linear direction in the "strafe"
            // [side-to-side] direction will require additional logic.
            telemetry.addData("Pitch Angle (degrees):", pitchAngleDegrees);
            telemetry.addData("Roll Angle (degrees): ", rollAngleDegrees);
            telemetry.addData("X Axis Balance Rate:  ", xAxisRate);
            telemetry.addData("X Axis Balance Rate:  ", yAxisRate);
        }
    }

    @Override
    public void stop() {
        navx_device.close();
    }
}
