package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transit extends SubsystemBase {
    public final Servo limitServo;
    public TransitState transitState;
    public final Servo transitServo;

    public Transit(HardwareMap hardwareMap, boolean isTeleop) {
        limitServo = hardwareMap.get(Servo.class, TransitConstants.limitServoName);
        this.transitServo = hardwareMap.get(Servo.class, TransitConstants.transitServoName);
        if (isTeleop) transitState = TransitState.CLOSE;
        else transitState = TransitState.OPEN;
    }

    public static enum TransitState {
        CLOSE(TransitConstants.limitServoClosePos, TransitConstants.transitServoClosePos),
        OPEN(TransitConstants.limitServoOpenPos, TransitConstants.transitServoOpenPos);

        final double limitServoPos, transitServoPos;

        TransitState(double limitServoPos, double transitServoPos) {
            this.limitServoPos = limitServoPos;
            this.transitServoPos = transitServoPos;
        }
    }


    public void setState(TransitState transitState) {
        this.transitState = transitState;
    }

    public TransitState getState() {
        return this.transitState;
    }

    @Override
    public void periodic() {
        limitServo.setPosition(transitState.limitServoPos);
        transitServo.setPosition(transitState.transitServoPos);
    }
}
