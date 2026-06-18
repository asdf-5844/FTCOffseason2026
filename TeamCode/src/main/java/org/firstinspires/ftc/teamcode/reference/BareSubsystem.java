package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.hardware.HardwareMap;

// Minimal subsystem skeleton

public class BareSubsystem {

    // Hardware goes here
    // private DcMotorEx motor;
    // private Servo servo;

    public BareSubsystem(HardwareMap hardwareMap) {
        // Initialize hardware here
        // motor = hardwareMap.get(DcMotorEx.class, "motorName");
        // servo = hardwareMap.get(Servo.class, "servoName");
    }

    public void update() {
        // Runs every loop, call subsystem.update()
    }

    public void stop() {
        // Stop motors here
        // motor.setPower(0);
    }
}