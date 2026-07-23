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
    private boolean useMovementStallProtection;
    private boolean stallProtectionArmed;
    private double minimumProgress;
    private double stallTime;
    private double minimumMovement;
    private double bestProgress;
    private double lastMovementX;
    private double lastMovementY;

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

    public AutoDriveCommand(
            Follower follower,
            PathChain pathChain,
            double minimumProgress,
            double stallTime,
            double minimumMovement
    ) {
        this(follower, pathChain, minimumProgress, stallTime);
        this.useMovementStallProtection = true;
        this.minimumMovement = minimumMovement;
    }

    @Override
    public void initialize() {
        timer.reset();
        stallTimer.reset();
        bestProgress = 0;
        stallProtectionArmed = false;
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
            if (!useMovementStallProtection) {
                stallTimer.reset();
            }
        }

        if (!useMovementStallProtection || bestProgress < minimumProgress) {
            return;
        }

        double currentX = follower.getPose().getX();
        double currentY = follower.getPose().getY();
        if (!stallProtectionArmed) {
            lastMovementX = currentX;
            lastMovementY = currentY;
            stallProtectionArmed = true;
            stallTimer.reset();
        } else if (Math.hypot(currentX - lastMovementX, currentY - lastMovementY) >= minimumMovement) {
            lastMovementX = currentX;
            lastMovementY = currentY;
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
                && (!useMovementStallProtection || stallProtectionArmed)
                && stallTimer.milliseconds() >= stallTime;
        return !follower.isBusy() || stalledNearEnd || timer.milliseconds() >= waitTime;
    }
}
