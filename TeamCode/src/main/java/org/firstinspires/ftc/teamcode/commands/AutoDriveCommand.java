package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoDriveCommand extends CommandBase {
    private static final double MIN_PROGRESS_DELTA = 0.005;

    private Follower follower;
    private PathChain pathChain;
    private double waitTime;
    private final ElapsedTime timer;
    private final ElapsedTime stallTimer;
    private boolean useStallProtection;
    private double minimumProgress;
    private double stallTime;
    private double bestProgress;

    public AutoDriveCommand(Follower follower, PathChain pathChain) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.waitTime = 30 * 1000;
        this.timer = new ElapsedTime();
        this.stallTimer = new ElapsedTime();
    }

    public AutoDriveCommand(Follower follower, PathChain pathChain, double waitTime) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.waitTime = waitTime;
        this.timer = new ElapsedTime();
        this.stallTimer = new ElapsedTime();
    }

    public AutoDriveCommand(
            Follower follower,
            PathChain pathChain,
            double minimumProgress,
            double stallTime
    ) {
        this(follower, pathChain);
        this.useStallProtection = true;
        this.minimumProgress = minimumProgress;
        this.stallTime = stallTime;
    }

    @Override
    public void initialize() {
        timer.reset();
        stallTimer.reset();
        bestProgress = 0;
        follower.followPath(pathChain);
    }

    @Override
    public void execute() {
        if (!useStallProtection) {
            return;
        }

        double currentProgress = follower.getCurrentTValue();
        if (currentProgress > bestProgress + MIN_PROGRESS_DELTA) {
            bestProgress = currentProgress;
            stallTimer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        follower.breakFollowing();
    }

    @Override
    public boolean isFinished() {
        boolean stalledNearEnd = useStallProtection
                && bestProgress >= minimumProgress
                && stallTimer.milliseconds() >= stallTime;
        return !follower.isBusy() || stalledNearEnd || timer.milliseconds() >= waitTime;
    }
}
