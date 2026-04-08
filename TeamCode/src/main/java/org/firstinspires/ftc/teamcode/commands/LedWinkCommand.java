package org.firstinspires.ftc.teamcode.commands;

import android.view.Window;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.led.Led;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

public class LedWinkCommand extends CommandBase {
    private Led led;
    private Shooter shooter;
    private Turret turret;
    private Drive.Alliance alliance;
    private Transit transit;
    private Intake intake;
    private ElapsedTime timer;

    public LedWinkCommand(Led led, Shooter shooter, Transit transit, Turret turret, Drive.Alliance alliance) {
        this.led = led;
        this.shooter = shooter;
        this.transit = transit;
        this.turret = turret;
        this.alliance = alliance;
        this.timer = new ElapsedTime();

        addRequirements(led);
    }

    @Override
    public void initialize() {
        this.timer.reset();
    }

    @Override
    public void execute() {
        if (!led.getAligned()){
            if (alliance == Drive.Alliance.RED) {
                led.setLedState(Led.LedState.REDRAINBOW);
            }
            else {
                led.setLedState(Led.LedState.BLUERAINBOW);
            }
        }

        else if (shooter.isShooterAtSetPoint() && turret.isAligned() && transit.getState() == Transit.TransitState.OPEN) {
            led.setLedState(Led.LedState.GREENFLICKER);
        }
        else if (shooter.isShooterAtSetPoint() && turret.isAligned()) {
            led.setLedState(Led.LedState.GREEN);
        }
        else {
            led.setLedState(Led.LedState.RED);
        }
    }

    /*@Override
    public void end(boolean interrupted) {
        led.setLedState(Led.LedState.YELLOW);
    }*/

    @Override
    public boolean isFinished() {
        return false;
    }
}
