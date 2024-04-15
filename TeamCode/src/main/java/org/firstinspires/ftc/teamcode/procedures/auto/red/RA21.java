package org.firstinspires.ftc.teamcode.procedures.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.controllers.common.utilities.Side;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.Team;
import org.firstinspires.ftc.teamcode.procedures.auto.DynamicAutoBase;

@Autonomous(name = "RA21")
public class RA21 extends DynamicAutoBase {
    @Override
    public void runOnStart () {
        team = Team.RED;
        side = Side.AUDIENCE;
    }
}
