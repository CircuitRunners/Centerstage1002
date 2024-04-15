package org.firstinspires.ftc.teamcode.controllers.common.utilities;

import static org.firstinspires.ftc.teamcode.controllers.common.utilities.PropLocation.LEFT;
import static org.firstinspires.ftc.teamcode.controllers.common.utilities.PropLocation.MIDDLE;
import static org.firstinspires.ftc.teamcode.controllers.common.utilities.PropLocation.RIGHT;
import static org.firstinspires.ftc.teamcode.controllers.common.utilities.Side.AUDIENCE;
import static org.firstinspires.ftc.teamcode.controllers.common.utilities.Side.BACKSTAGE;
import static org.firstinspires.ftc.teamcode.controllers.common.utilities.Team.BLUE;
import static org.firstinspires.ftc.teamcode.controllers.common.utilities.Team.RED;


public enum CrossBounds {
    RED_STAGE_LEFT(RED, BACKSTAGE, LEFT),
    RED_STAGE_MIDDLE(RED, BACKSTAGE, MIDDLE),
    RED_STAGE_RIGHT(RED, BACKSTAGE, RIGHT),
    RED_AUDIENCE_LEFT(RED, AUDIENCE, LEFT),
    RED_AUDIENCE_MIDDLE(RED, AUDIENCE, MIDDLE),
    RED_AUDIENCE_RIGHT(RED, AUDIENCE, RIGHT),
    BLUE_STAGE_LEFT(BLUE, BACKSTAGE, LEFT),
    BLUE_STAGE_MIDDLE(BLUE, BACKSTAGE, MIDDLE),
    BLUE_STAGE_RIGHT(BLUE, BACKSTAGE, RIGHT),
    BLUE_AUDIENCE_LEFT(BLUE, AUDIENCE, LEFT),
    BLUE_AUDIENCE_MIDDLE(BLUE, AUDIENCE, MIDDLE),
    BLUE_AUDIENCE_RIGHT(BLUE, AUDIENCE, RIGHT);

    private final PropLocation propLocation;
    private final Team team;
    private final Side side;

    CrossBounds(Team team, Side side, PropLocation propLocation) {
        this.propLocation = propLocation;
        this.side = side;
        this.team = team;
    }

    public PropLocation getPropLocation() {
        return propLocation;
    }

    public Team getTeam() {
        return team;
    }

    public Side getSide() {
        return side;
    }

    public static CrossBounds findCrossBound(Team team, Side side, PropLocation propLocation) {
        for (CrossBounds crossBound : CrossBounds.values()) {
            if (crossBound.getTeam() == team && crossBound.getSide() == side && crossBound.getPropLocation() == propLocation) {
                return crossBound;
            }
        }
        return null; // or throw new IllegalArgumentException("No matching CrossBound found");
    }
}