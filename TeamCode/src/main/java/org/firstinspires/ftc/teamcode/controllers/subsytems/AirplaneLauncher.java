package org.firstinspires.ftc.teamcode.controllers.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.controllers.common.utilities.ServoModule;

@Config
public class AirplaneLauncher extends SubsystemBase {

    // Declarations
    public enum LauncherPosition {
        SPRUNG(0.156), // Replace with the actual servo value for the SPRUNG position
        LAUNCH(0.03); // Replace with the actual servo value for the LAUNCH position

        public final double position;

        LauncherPosition(double position){
            this.position = position;
        }
    }

    public static double
            LAUNCHER_SPRUNG = LauncherPosition.SPRUNG.position,
            LAUNCHER_LAUNCH = LauncherPosition.LAUNCH.position;



    private ServoImplEx launcherServo;

    private String servoHardwareMapName = "airplaneLauncher";
    private HardwareMap hardwareMap;
    private ServoModule testingModule;

    public AirplaneLauncher(HardwareMap hardwareMap){

        // Retrieve the servo from the hardware map
        launcherServo = hardwareMap.get(ServoImplEx.class, servoHardwareMapName);

        this.hardwareMap = hardwareMap;

        // Initialize to the SPRUNG position
        cock();
    }

    // Normal use functions

    // Function in all subsystems to take in all relevant machine states
    // E.g. for airplane launcher open and close bindings
    public void processInput (boolean launch_button, boolean reset_button) {
        if (launch_button){
            launch();
        }
        else if (reset_button) {
            cock();
        }
    }

    // Method to set the launcher to the LAUNCH position
    public void launch() {
        launcherServo.setPosition(LAUNCHER_LAUNCH);
    }
    // Method to set the launcher to the SPRUNG position
    public void cock() {
        launcherServo.setPosition(LAUNCHER_SPRUNG);
    }

    // Special methods

    // Method to get the current position of the launcher servo
    public double getPosition() {
        return launcherServo.getPosition();
    }
    // Method to disable the PWM signal to the servo
    public void tuningModeOn(GamepadEx gamepad) {
        testingModule = new ServoModule(this.hardwareMap, servoHardwareMapName, gamepad);
    }
    public void tuningModeOn(GamepadEx gamepad, double initialPosition) {
        testingModule = new ServoModule(this.hardwareMap, servoHardwareMapName, gamepad, initialPosition);
    }
    // Method to re-enable the PWM signal to the servo
    public void tuningModeOff() {
        testingModule = null;
    }
}