package org.firstinspires.ftc.teamcode.procedures.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.controllers.common.utilities.Side;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.Team;
import org.firstinspires.ftc.teamcode.procedures.auto.DynamicAutoBase;

@Autonomous(name = "BA21")
public class BA21 extends DynamicAutoBase {
    @Override
    public void runOnStart () {
        team = Team.BLUE;
        side = Side.AUDIENCE;
    }
}
