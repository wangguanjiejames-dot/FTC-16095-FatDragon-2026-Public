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
        this.alliance = Drive.Alliance.BLUE;

        follower.setStartingPose(
                new Pose(25.698, 128.559, Math.toRadians(142))
        );

        Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(25.698, 128.559),
                        new Pose(50.598, 118.339)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(142),
                Math.toRadians(-90)
        ).build();

        Path2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(50.598, 118.339),
                        new Pose(79.844, 45.105),
                        new Pose(21.769, 59.564)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-90),
                Math.toRadians(180)
        ).build();

        Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(21.769, 59.564),
                        new Pose(58.421, 74.681)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(-90)
        ).build();

        Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(58.421, 74.681),
                        new Pose(16.580, 64.269)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-90),
                Math.toRadians(180)
        ).build();

        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(16.580, 64.269),
                        new Pose(12.860, 55.380)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(135)
        ).build();

        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(12.860, 55.380),
                        new Pose(58.494, 74.701)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(135),
                Math.toRadians(-90)
        ).build();

        Path7 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(58.494, 74.701),
                        new Pose(16.485, 64.043)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-90),
                Math.toRadians(180)
        ).build();

        Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(16.485, 64.043),
                        new Pose(13.086, 55.493)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(135)
        ).build();

        Path9 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(13.086, 55.493),
                        new Pose(57.935, 74.329)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(135),
                Math.toRadians(-90)
        ).build();

        Path10 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(57.935, 74.329),
                        new Pose(71.907, 84.947),
                        new Pose(57.143, 79.369),
                        new Pose(21.526, 83.477)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-90),
                Math.toRadians(180)
        ).build();

        Path11 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(21.526, 83.477),
                        new Pose(54.396, 84.202)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(131)
        ).build();

        Path12 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(54.396, 84.202),
                        new Pose(63.492, 22.882),
                        new Pose(20.242, 35.058)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(131),
                Math.toRadians(180)
        ).build();

        Path13 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(20.242, 35.058),
                        new Pose(60.990, 73.809)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(-90)
        ).build();

        Path14 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(60.990, 73.809),
                        new Pose(60.602, 62.898)
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(-90),
                Math.toRadians(180)
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
                                new AutoDriveCommand(follower, Path4),
                                intakeDuringPathAndWait(Path5, 150),
                                new AutoDriveCommand(follower, Path6),
                                shootFor(900),
                                new AutoDriveCommand(follower, Path7),
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
