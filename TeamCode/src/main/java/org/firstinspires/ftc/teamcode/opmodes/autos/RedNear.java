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

@Autonomous(name = "Red Near", group = "Auto")
public class RedNear extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private Turret turret;
    private Vision vision;
    private Drive.Alliance alliance;
    private static final String AUTO_BUILD = "Red18LeaveNew-P5-Debug-20260718";
    private Command autoCommand;
    private String autoStep = "Initializing";
    private long autoLoopCount;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14;

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
        this.alliance = Drive.Alliance.RED;

        follower.setStartingPose(
                new Pose(118.302, 128.559, Math.toRadians(38))
        );

        Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(118.302, 128.559),
                        new Pose(93.402, 118.339)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(38),
                Math.toRadians(90)
        ).build();

        Path2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(93.402, 118.339),
                        new Pose(66.691, 72.652),
                        new Pose(79.413, 50.645),
                        new Pose(122.231, 59.564)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(90),
                Math.toRadians(0)
        ).build();

        Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(122.231, 59.564),
                        new Pose(85.579, 74.681)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(0),
                Math.toRadians(90)
        ).build();

        Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(85.579, 74.681),
                        new Pose(125.744, 65.014)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(90),
                Math.toRadians(0)
        ).build();

        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(125.744, 65.014),
                        new Pose(131.140, 54.263)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(0),
                Math.toRadians(45)
        ).build();

        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(131.140, 54.263),
                        new Pose(85.506, 74.701)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(45),
                Math.toRadians(90)
        ).build();

        Path7 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(85.506, 74.701),
                        new Pose(125.466, 65.160)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(90),
                Math.toRadians(0)
        ).build();

        Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(125.466, 65.160),
                        new Pose(131.100, 54.376)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(0),
                Math.toRadians(45)
        ).build();

        Path9 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(131.100, 54.376),
                        new Pose(86.065, 74.329)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(45),
                Math.toRadians(90)
        ).build();

        Path10 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(86.065, 74.329),
                        new Pose(72.093, 84.947),
                        new Pose(86.857, 79.369),
                        new Pose(122.474, 83.477)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(90),
                Math.toRadians(0)
        ).build();

        Path11 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(122.474, 83.477),
                        new Pose(89.604, 84.202)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(0),
                Math.toRadians(49)
        ).build();

        Path12 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(89.604, 84.202),
                        new Pose(80.508, 22.882),
                        new Pose(123.758, 35.058)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(49),
                Math.toRadians(0)
        ).build();

        Path13 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(123.758, 35.058),
                        new Pose(83.010, 73.809)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(0),
                Math.toRadians(90)
        ).build();

        Path14 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(83.010, 73.809),
                        new Pose(83.398, 62.898)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(90),
                Math.toRadians(0)
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
                                new AutoDriveCommand(follower, Path4, 0.93, 200),
                                intakeDuringPathAndWait(Path5, 150),
                                new AutoDriveCommand(follower, Path6),
                                shootFor(900),
                                new AutoDriveCommand(follower, Path7, 0.93, 200),
                                intakeDuringPathAndWait(Path8, 150),
                                new AutoDriveCommand(follower, Path9),
                                shootFor(900),
                                intakeDuringPath(Path10),
                                new AutoDriveCommand(follower, Path11),
                                shootFor(900),
                                intakeDuringPath(Path12),
                                new AutoDriveCommand(follower, Path13),
                                shootFor(900),
                                new AutoDriveCommand(follower, Path14)
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
