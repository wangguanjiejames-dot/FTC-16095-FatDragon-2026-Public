package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterAlignCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.TurretAlignCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

@Autonomous(name = "Blue Far", group = "Auto")
public class BlueFar extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private Turret turret;
    private Vision vision;
    private Drive.Alliance alliance;
    private Command autoCommand;

    public PathChain Path1, Path2, Path3, Path4, Path5;

    private Command shootFor(long milliseconds) {
        return new ParallelDeadlineGroup(
                new WaitCommand(milliseconds),
                new TransitCommand(shooter, transit, intake)
        ).andThen(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.STOP)));
    }

    private Command intakeDuringPath(PathChain path) {
        return new ParallelDeadlineGroup(
                new AutoDriveCommand(follower, path),
                new IntakeCommand(intake, transit)
        ).andThen(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.STOP)));
    }

    @Override
    public void initialize() {
        this.follower = Constants.createFollower(hardwareMap);
        this.intake = new Intake(hardwareMap);
        this.shooter = new Shooter(hardwareMap);
        this.transit = new Transit(hardwareMap, false);
        this.turret = new Turret(hardwareMap);
        this.vision = new Vision(hardwareMap);
        this.alliance = Drive.Alliance.RED;

        follower.setStartingPose(
                new Pose(72, 8, Math.toRadians(90))
        );

        Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(56.373, 7.441),
                        new Pose(27.198, 21.609)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(-90)
        ).build();

        Path2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(27.198, 21.609),
                        new Pose(13.937, 21.649),
                        new Pose(7.060, 26.110)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-90),
                Math.toRadians(-90)
        ).build();

        Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(7.060, 26.110),
                        new Pose(7.555, 8.981)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-90),
                Math.toRadians(-90)
        ).build();

        Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(7.555, 8.981),
                        new Pose(65.573, 19.374)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-90),
                Math.toRadians(180)
        ).build();

        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(65.573, 19.374),
                        new Pose(31.590, 15.036)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(90)
        ).build();

        autoCommand = new ParallelCommandGroup(
                new TurretAlignCommand(follower, turret, alliance, vision),
                new ShooterAlignCommand(follower, shooter, transit, alliance),
                new SequentialCommandGroup(
                        shootFor(900),
                        new AutoDriveCommand(follower, Path1),
                        new AutoDriveCommand(follower, Path2),
                        intakeDuringPath(Path3),
                        new AutoDriveCommand(follower, Path4),
                        shootFor(900),
                        new AutoDriveCommand(follower, Path5)
                )
        );
        schedule(autoCommand);
    }

    @Override
    public void run() {
        follower.update();
        CommandScheduler.getInstance().run();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Shooter at Setpoint: ", shooter.isShooterAtSetPoint());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Path T", follower.getCurrentTValue());
        telemetry.addData("Auto Scheduled", CommandScheduler.getInstance().isScheduled(autoCommand));
        telemetry.update();
    }
}
