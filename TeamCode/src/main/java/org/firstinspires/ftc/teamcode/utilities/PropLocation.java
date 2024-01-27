package org.firstinspires.ftc.teamcode.utilities;

public enum PropLocation {

    LEFT("LEFT"),
    MIDDLE("MIDDLE"),
    RIGHT("RIGHT");

    private final String location;

    PropLocation(String location) {
        this.location = location;
    }

    public String getLocation() {
        return location;
    }
}
