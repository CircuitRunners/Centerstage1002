package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.Gamepad;

public class PseudoGamepad {
    public int id;
    public int user;
    public float left_stick_x;
    public float left_stick_y;
    public float right_stick_x;
    public float right_stick_y;
    public float left_trigger;
    public float right_trigger;
    public boolean dpad_up;
    public boolean dpad_down;
    public boolean dpad_left;
    public boolean dpad_right;
    public boolean a;
    public boolean b;
    public boolean x;
    public boolean y;
    public boolean guide;
    public boolean start;
    public boolean back;
    public boolean left_bumper;
    public boolean right_bumper;
    public boolean left_stick_button;
    public boolean right_stick_button;

    public boolean cross, square, triangle, ps, share, circle, options, touchpad;

    public Gamepad innerGamepad;

    public void from(Gamepad gPad) {
        innerGamepad = gPad;
    }

    public void rumble (int msDuration) {
        innerGamepad.rumble(msDuration);
    }

    public static PseudoGamepad parse(String str) {
        PseudoGamepad gamepad = new PseudoGamepad();

        // Split the string into an array of parts
        String[] parts = str.split(" ");

        // Parse the ID, user, and joystick values
        gamepad.id = Integer.parseInt(parts[1]);
        gamepad.user = Integer.parseInt(parts[3]);
        gamepad.left_stick_x = Float.parseFloat(parts[5]);
        gamepad.left_stick_y = Float.parseFloat(parts[7]);
        gamepad.right_stick_x = Float.parseFloat(parts[9]);
        gamepad.right_stick_y = Float.parseFloat(parts[11]);
        gamepad.left_trigger = Float.parseFloat(parts[13]);
        gamepad.right_trigger = Float.parseFloat(parts[15]);

        // Parse the button states
        for (int i = 16; i < parts.length; i++) {
            switch (parts[i]) {
                case "dpad_up":
                    gamepad.dpad_up = true;
                    break;
                case "dpad_down":
                    gamepad.dpad_down = true;
                    break;
                case "dpad_left":
                    gamepad.dpad_left = true;
                    break;
                case "dpad_right":
                    gamepad.dpad_right = true;
                    break;
                case "cross":
                    gamepad.cross = true;
                    break;
                case "circle":
                    gamepad.circle = true;
                    break;
                case "square":
                    gamepad.square = true;
                    break;
                case "triangle":
                    gamepad.triangle = true;
                    break;
                case "ps":
                    gamepad.ps = true;
                    break;
                case "share":
                    gamepad.share = true;
                    break;
                case "options":
                    gamepad.options = true;
                    break;
                case "touchpad":
                    gamepad.touchpad = true;
                    break;
                case "left_bumper":
                    gamepad.left_bumper = true;
                    break;
                case "right_bumper":
                    gamepad.right_bumper = true;
                    break;
                case "left":
                    if (parts[i + 1].equals("stick")) {
                        gamepad.left_stick_button = true;
                        i++; // Skip the next part
                    }
                    break;
                case "right":
                    if (parts[i + 1].equals("stick")) {
                        gamepad.right_stick_button = true;
                        i++; // Skip the next part
                    }
                    break;
            }
        }

        return gamepad;
    }
}
