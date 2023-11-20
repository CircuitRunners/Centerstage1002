package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class AirplaneLauncher extends SubsystemBase {

    // Declarations
    public enum LauncherPosition {
        SPRUNG(0.72), // Replace with the actual servo value for the SPRUNG position
        LAUNCH(0.18); // Replace with the actual servo value for the LAUNCH position

        public final double position;

        LauncherPosition(double position){
            this.position = position;
        }
    }

    private ServoImplEx launcherServo;

    public AirplaneLauncher(HardwareMap hardwareMap){
        // Retrieve the servo from the hardware map
        launcherServo = hardwareMap.get(ServoImplEx.class, "airplaneLauncher");

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
        launcherServo.setPosition(LauncherPosition.LAUNCH.position);
    }
    // Method to set the launcher to the SPRUNG position
    public void cock() {
        launcherServo.setPosition(LauncherPosition.SPRUNG.position);
    }

    // Special methods

    // Method to get the current position of the launcher servo
    public double getPosition() {
        return launcherServo.getPosition();
    }
    // Method to disable the PWM signal to the servo
    public void tuningModeOn() {
        launcherServo.setPwmDisable();
    }
    // Method to re-enable the PWM signal to the servo
    public void tuningModeOff() {
        launcherServo.setPwmEnable();
    }
}