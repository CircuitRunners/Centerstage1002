package org.firstinspires.ftc.teamcode.utilities;

public class Utilities {
    // Threshold for the debounce method
    public static final double DEBOUNCE_THRESHOLD = 0.05;
    public static final double CLOSE_THRESHOLD = 0.10;
    public static final double CLOSE_TO_ONE = 1 - CLOSE_THRESHOLD - DEBOUNCE_THRESHOLD;
    public static final double CLOSE_TO_ZERO = CLOSE_THRESHOLD + DEBOUNCE_THRESHOLD;
    public static final double DEBOUNCE_FROM_ONE = 1 - DEBOUNCE_THRESHOLD;

    // Make sure that all inputs are debounced, on purpose
    // Not just a random value the controller gets stuck on!
    // Its static so you can use it without making a Utilities class
    // Also it means there are no "objects" just one across everything
    public static boolean debounce(double input) {
        return Math.abs(input) > DEBOUNCE_THRESHOLD;
    }

    public static double squash(double input) {
        if (debounce(input)) {
            return input;
        } else {
            return 0;
        }
    }

    public static double diff(double input, double input2) {
        return input - input2;
    }

    public static double squash(double input, double input2) {
        input = squash(input);
        input2 = squash(input2);

        return input - input2;
    }

    public static double neg(double input) {
        return input * -1;
    }
}