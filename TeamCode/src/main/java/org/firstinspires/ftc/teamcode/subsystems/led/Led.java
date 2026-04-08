package org.firstinspires.ftc.teamcode.subsystems.led;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


public class Led extends SubsystemBase {
    RevBlinkinLedDriver led;
    private boolean aligned;

    public boolean getAligned() {
        return aligned;
    }

    public enum LedState {
        YELLOW(RevBlinkinLedDriver.BlinkinPattern.YELLOW),
        GREEN(RevBlinkinLedDriver.BlinkinPattern.GREEN),
        RED(RevBlinkinLedDriver.BlinkinPattern.RED),
        REDCHASE(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED),
        BLUECHASE(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE),
        GREENCHASE(RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE),
        GREENFLICKER(RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE),
        BLUERAINBOW(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE),
        REDRAINBOW(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE),
        RAINBOW(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);


        public RevBlinkinLedDriver.BlinkinPattern pattern;

        LedState(RevBlinkinLedDriver.BlinkinPattern pattern) {
            this.pattern = pattern;
        }
    }

    public LedState ledState;

    public Led(HardwareMap hardwareMap) {
        this.led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        ledState = LedState.GREEN;
        this.aligned = false;
    }

    public void setLedState(LedState ledState) {
        this.ledState = ledState;
    }

    public void setAligned(boolean aligned) {
        this.aligned = aligned;
    }

    @Override
    public void periodic() {
        led.setPattern(ledState.pattern);
    }
}
