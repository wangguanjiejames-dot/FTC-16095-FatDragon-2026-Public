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

@Autonomous(name = "Blue Near", group = "Auto")
public class BlueNear extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private Turret turret;
    private Vision vision;
    private Drive.Alliance alliance;
    private static final String AUTO_BUILD = "Blue18LeaveNew-P5-Debug-20260718";
    private Command autoCommand;
    private String autoStep = "Initializing";
    private long autoLoopCount;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13;

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

    private Command intakeDuringPathAndWait(PathChain path, long waitMs) {
        return new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        new AutoDriveCommand(follower, path),
                        new WaitCommand(waitMs)
                ),
                new IntakeCommand(intake, transit)
        ).andThen(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.STOP)));
    }

    private Command intakeDuringTimedPath(PathChain path, double driveTimeout) {
        return new ParallelDeadlineGroup(
                new AutoDriveCommand(follower, path, driveTimeout),
                new IntakeCommand(intake, transit)
        ).andThen(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.STOP)));
    }

    private Command intakeDuringTimedPathAndWait(PathChain path, double driveTimeout, long waitMs) {
        return new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        new AutoDriveCommand(follower, path, driveTimeout),
                        new WaitCommand(waitMs)
                ),
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
        this.alliance = Drive.Alliance.BLUE;

        follower.setStartingPose(
                new Pose(25.631, 128.759, Math.toRadians(142))
        );

        Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(25.631, 128.759),
                        new Pose(54.137, 104.926)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(142),
                Math.toRadians(-90)
        ).build();

        Path2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(54.137, 104.926),
                        new Pose(57.303, 57.773),
                        new Pose(21.583, 59.937)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-90),
                Math.toRadians(180)
        ).build();

        Path3 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(21.583, 59.937),
                        new Pose(50.199, 78.636),
                        new Pose(50.783, 93.982)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(-90)
        ).build();

        Path4 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(50.783, 93.982),
                        new Pose(46.960, 76.127),
                        new Pose(16.027, 64.881)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-90),
                Math.toRadians(180)
        ).build();

        Path5 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(16.027, 64.881),
                        new Pose(15.462, 56.072),
                        new Pose(11.736, 50.298)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(135)
        ).build();

        Path6 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(11.736, 50.298),
                        new Pose(47.243, 70.113),
                        new Pose(50.856, 93.516)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(135),
                Math.toRadians(-90)
        ).build();

        Path7 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(50.856, 93.516),
                        new Pose(46.525, 75.005),
                        new Pose(15.926, 64.974)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-90),
                Math.toRadians(180)
        ).build();

        Path8 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(15.926, 64.974),
                        new Pose(14.910, 54.649),
                        new Pose(11.943, 50.374)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(135)
        ).build();

        Path9 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(11.943, 50.374),
                        new Pose(48.807, 71.534),
                        new Pose(50.790, 93.626)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(135),
                Math.toRadians(-90)
        ).build();

        Path10 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(50.790, 93.626),
                        new Pose(53.601, 80.218),
                        new Pose(62.072, 84.713),
                        new Pose(19.477, 83.850)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-90),
                Math.toRadians(180)
        ).build();

        Path11 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(19.477, 83.850),
                        new Pose(49.366, 84.016)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(180)
        ).build();

        Path12 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(49.366, 84.016),
                        new Pose(51.570, 32.569),
                        new Pose(18.752, 35.431)
                )
        ).setTangentHeadingInterpolation().build();

        Path13 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(18.752, 35.431),
                        new Pose(60.990, 73.809)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(-90)
        ).build();

        autoCommand = new ParallelCommandGroup(
                        new TurretAlignCommand(follower, turret, alliance, vision),
                        new ShooterAlignCommand(follower, shooter, transit, alliance),
                        new SequentialCommandGroup(
                                new AutoDriveCommand(follower, Path1),
                                shootFor(900),
                                intakeDuringPath(Path2),
                                new AutoDriveCommand(follower, Path3),
                                shootFor(900),
                                new AutoDriveCommand(follower, Path4, 250),
                                new InstantCommand(() -> autoStep = "Path5 intake"),
                                intakeDuringTimedPath(Path5, 400),
                                new InstantCommand(() -> autoStep = "Path5 wait"),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(200),
                                        new IntakeCommand(intake, transit)
                                ),
                                new InstantCommand(() -> autoStep = "Path6"),   
                                intakeDuringPath(Path6),
                                shootFor(900),
                                new AutoDriveCommand(follower, Path7, 250),
                                new InstantCommand(() -> autoStep = "Path8 intake"),
                                intakeDuringTimedPath(Path8, 400),
                                new InstantCommand(() -> autoStep = "Path8 wait"),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(200),
                                        new IntakeCommand(intake, transit)
                                ),
                                new InstantCommand(() -> autoStep = "Path9"),
                                intakeDuringPath(Path9),
                                shootFor(900),
                                intakeDuringPath(Path10),
                                new AutoDriveCommand(follower, Path11),
                                shootFor(900),
                                intakeDuringPath(Path12),
                                new AutoDriveCommand(follower, Path13),
                                shootFor(900)
                        )
        );
        schedule(autoCommand);
    }

    @Override
    public void run() {
        autoLoopCount++;
        follower.update();
        CommandScheduler.getInstance().run();

        telemetry.addData("Vision Pose: ", vision.getVisionPose());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData(
                "Shooter at Setpoint: ",
                shooter.isShooterAtSetPoint()
        );
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Heading Error", follower.getHeadingError());
        telemetry.addData("Path T", follower.getCurrentTValue());
        telemetry.addData("Auto Step", autoStep);
        telemetry.addData("Auto Build", AUTO_BUILD);
        telemetry.addData("Auto Scheduled", CommandScheduler.getInstance().isScheduled(autoCommand));
        telemetry.addData("Auto Loop", autoLoopCount);
        telemetry.update();
    }
}
