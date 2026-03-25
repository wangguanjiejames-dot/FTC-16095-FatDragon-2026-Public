package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * Input probing OpMode for experimenting with a Logitech Extreme 3D Pro
 * (or any other nonstandard controller) through the normal FTC gamepad API.
 *
 * This OpMode does not control robot hardware. It only reports which gamepad1
 * fields are changing so the observed mapping can be documented and reused later.
 */
@TeleOp(name = "Extreme3DProbe", group = "Diagnostics")
public class Extreme3DProbe extends OpMode {
    private static final double ANALOG_CHANGE_THRESHOLD = 0.02;

    private double previousLeftStickX;
    private double previousLeftStickY;
    private double previousRightStickX;
    private double previousRightStickY;
    private double previousLeftTrigger;
    private double previousRightTrigger;

    private boolean previousA;
    private boolean previousB;
    private boolean previousX;
    private boolean previousY;
    private boolean previousDpadUp;
    private boolean previousDpadDown;
    private boolean previousDpadLeft;
    private boolean previousDpadRight;
    private boolean previousLeftBumper;
    private boolean previousRightBumper;
    private boolean previousBack;
    private boolean previousStart;
    private boolean previousGuide;
    private boolean previousLeftStickButton;
    private boolean previousRightStickButton;

    @Override
    public void init() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        captureCurrentState();
        telemetry.addLine("Extreme3DProbe ready");
        telemetry.addLine("Press INIT, then START, then move one control at a time.");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addLine("Extreme3DProbe - gamepad1 input probe");
        telemetry.addLine("------------------------------------");
        telemetry.addLine("Move one joystick control at a time");
        telemetry.addLine("Press one button at a time");
        telemetry.addLine("Record which FTC field changes");
        telemetry.addLine("");

        telemetry.addLine("Legend: [*] changed this loop, [ ] unchanged");
        telemetry.addLine("");

        telemetry.addLine("Analog Inputs");
        telemetry.addLine(formatAnalog("left_stick_x", gamepad1.left_stick_x, previousLeftStickX));
        telemetry.addLine(formatAnalog("left_stick_y", gamepad1.left_stick_y, previousLeftStickY));
        telemetry.addLine(formatAnalog("right_stick_x", gamepad1.right_stick_x, previousRightStickX));
        telemetry.addLine(formatAnalog("right_stick_y", gamepad1.right_stick_y, previousRightStickY));
        telemetry.addLine(formatAnalog("left_trigger", gamepad1.left_trigger, previousLeftTrigger));
        telemetry.addLine(formatAnalog("right_trigger", gamepad1.right_trigger, previousRightTrigger));
        telemetry.addLine("");

        telemetry.addLine("Buttons");
        telemetry.addLine(formatBoolean("a", gamepad1.a, previousA));
        telemetry.addLine(formatBoolean("b", gamepad1.b, previousB));
        telemetry.addLine(formatBoolean("x", gamepad1.x, previousX));
        telemetry.addLine(formatBoolean("y", gamepad1.y, previousY));
        telemetry.addLine(formatBoolean("left_bumper", gamepad1.left_bumper, previousLeftBumper));
        telemetry.addLine(formatBoolean("right_bumper", gamepad1.right_bumper, previousRightBumper));
        telemetry.addLine(formatBoolean("left_stick_button", gamepad1.left_stick_button, previousLeftStickButton));
        telemetry.addLine(formatBoolean("right_stick_button", gamepad1.right_stick_button, previousRightStickButton));
        telemetry.addLine("");

        telemetry.addLine("D-pad");
        telemetry.addLine(formatBoolean("dpad_up", gamepad1.dpad_up, previousDpadUp));
        telemetry.addLine(formatBoolean("dpad_down", gamepad1.dpad_down, previousDpadDown));
        telemetry.addLine(formatBoolean("dpad_left", gamepad1.dpad_left, previousDpadLeft));
        telemetry.addLine(formatBoolean("dpad_right", gamepad1.dpad_right, previousDpadRight));
        telemetry.addLine("");

        telemetry.addLine("System Buttons");
        telemetry.addLine(formatBoolean("back", gamepad1.back, previousBack));
        telemetry.addLine(formatBoolean("start", gamepad1.start, previousStart));
        telemetry.addLine(formatBoolean("guide", gamepad1.guide, previousGuide));
        telemetry.addLine("");

        telemetry.addData("gamepad1.atRest()", gamepad1.atRest());
        telemetry.update();

        captureCurrentState();
    }

    /**
     * Formats an analog field and highlights meaningful changes to reduce noise.
     */
    private String formatAnalog(String label, double currentValue, double previousValue) {
        boolean changed = Math.abs(currentValue - previousValue) >= ANALOG_CHANGE_THRESHOLD;
        return String.format(
                Locale.US,
                "[%s] %-18s % .3f",
                changed ? "*" : " ",
                label,
                currentValue
        );
    }

    /**
     * Formats a button field and highlights any change in pressed state.
     */
    private String formatBoolean(String label, boolean currentValue, boolean previousValue) {
        boolean changed = currentValue != previousValue;
        return String.format(
                Locale.US,
                "[%s] %-18s %s",
                changed ? "*" : " ",
                label,
                currentValue ? "PRESSED" : "released"
        );
    }

    /**
     * Stores the current loop values so the next loop can detect changes.
     */
    private void captureCurrentState() {
        previousLeftStickX = gamepad1.left_stick_x;
        previousLeftStickY = gamepad1.left_stick_y;
        previousRightStickX = gamepad1.right_stick_x;
        previousRightStickY = gamepad1.right_stick_y;
        previousLeftTrigger = gamepad1.left_trigger;
        previousRightTrigger = gamepad1.right_trigger;

        previousA = gamepad1.a;
        previousB = gamepad1.b;
        previousX = gamepad1.x;
        previousY = gamepad1.y;
        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;
        previousDpadLeft = gamepad1.dpad_left;
        previousDpadRight = gamepad1.dpad_right;
        previousLeftBumper = gamepad1.left_bumper;
        previousRightBumper = gamepad1.right_bumper;
        previousBack = gamepad1.back;
        previousStart = gamepad1.start;
        previousGuide = gamepad1.guide;
        previousLeftStickButton = gamepad1.left_stick_button;
        previousRightStickButton = gamepad1.right_stick_button;
    }
}
