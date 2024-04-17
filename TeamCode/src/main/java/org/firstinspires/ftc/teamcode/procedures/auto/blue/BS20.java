package org.firstinspires.ftc.teamcode.procedures.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.controllers.common.utilities.Side;
import org.firstinspires.ftc.teamcode.controllers.common.utilities.Team;
import org.firstinspires.ftc.teamcode.procedures.auto.DynamicAutoBase;

@Autonomous(name = "BS20")
public class BS20 extends DynamicAutoBase {
    @Override
    public void runOnStart () {
        team = Team.BLUE;
        side = Side.BACKSTAGE;
    }
}
