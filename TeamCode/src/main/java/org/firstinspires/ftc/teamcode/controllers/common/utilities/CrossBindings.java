package org.firstinspires.ftc.teamcode.controllers.common.utilities;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class CrossBindings {

    // PLAYSTATION BINDINGS FOR THE CONTROLLERS!
    public static final GamepadKeys.Button circle = GamepadKeys.Button.B;
    public static final GamepadKeys.Button cross = GamepadKeys.Button.A;
    public static final GamepadKeys.Button triangle = GamepadKeys.Button.Y;
    public static final GamepadKeys.Button square = GamepadKeys.Button.X;

    public static final GamepadKeys.Button share = GamepadKeys.Button.BACK;
    public static final GamepadKeys.Button options = GamepadKeys.Button.START;

    // Roadrunner
    public static double halfPI = Math.PI/2.0;
    public static double rotationConstant = Math.toRadians(90);

    // Vision Stuff
    public static final int BUFFER_SIZE = 100;
    public static final int ZONE_COUNT = 3;
    public static final double TOP_CUTOFF_PIXELS_PROPORTION = 0.3;
    public static final double BOTTOM_CUTOFF_PIXELS_PROPORTION = 0.05;
    public static final int MIN_AREA = 300;

    public static final PropLocation DEFAULT_PROP_LOCATION = PropLocation.RIGHT;
}