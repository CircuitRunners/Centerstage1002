package org.firstinspires.ftc.teamcode.controllers.auto.pedrocommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.PathChain;

public class FollowPath extends CommandBase {
    private final Follower m_follower;
    private Path m_path = null;
    private PathChain m_pathChain = null;

    public FollowPath(Follower follower, Path path) {
        m_follower = follower;
        m_path = path;
    }

    public FollowPath(Follower follower, PathChain path) {
        m_follower = follower;
        m_pathChain = path;
    }

    @Override
    public void initialize() {
        if (m_path != null) {
            m_follower.followPath(m_path);
        } else {
            m_follower.followPath(m_pathChain);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return !m_follower.isBusy();
    }
}
