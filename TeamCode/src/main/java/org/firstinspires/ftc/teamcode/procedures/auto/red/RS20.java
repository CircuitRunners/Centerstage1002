package org.firstinspires.ftc.teamcode.procedures.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.controllers.common.utilities.Side;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.Team;
import org.firstinspires.ftc.teamcode.procedures.auto.DynamicAutoBase;

@Autonomous(name = "RS20")
public class RS20 extends DynamicAutoBase {
    @Override
    public void runOnStart () {
        team = Team.RED;
        side = Side.BACKSTAGE;
    }
}
