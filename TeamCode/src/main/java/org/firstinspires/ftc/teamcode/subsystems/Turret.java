package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Turret {
    private final DcMotorEx turret;
    private final TouchSensor turretLimitSwitch; // Magnetic Limit Switch
    private boolean wasPressed = false;

    private double targetAngle = 0;

    // Tune PID!!!
    public static double kp = 0.03;
    public static double kd = 0.0005;

    public static final double GOAL_X = -72.0; // tune offset
    public static final double GOAL_Y = -72.0; // tune offset
    private static final double MIN_ANGLE = -112;
    private static final double MAX_ANGLE = 132;

    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turretLimitSwitch = hardwareMap.get(TouchSensor.class, "homeSwitch");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotor.Direction.FORWARD);
    }

    public double getCurrentAngle() {
        // encoder ticks to degrees
        int ticks = turret.getCurrentPosition();
        return ticks / 1281.67 * 360.0;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getError() {
        return targetAngle - getCurrentAngle();
    }

    public void setTargetAngle(double angle) {
        // LIMITS
        angle = normalizeDegrees(angle);

        if (angle > MAX_ANGLE) {
            angle = MAX_ANGLE;
        }

        if (angle < MIN_ANGLE) {
            angle = MIN_ANGLE;
        }

        // save target
        targetAngle = angle;
    }

    public void update() {
        // compare target vs current
        // set motor power
        double currentAngle = getCurrentAngle();
        double error = normalizeDegrees(targetAngle - currentAngle);

        double power = 0;
        power += error * kp;


        if (Math.abs(error) < 1) {
            power = 0;
        }

        // clamp, speed limits
        if (power > 0.6) {
            power = 0.6;
        }
        if (power < -0.6) {
            power = -0.6;
        }

        turret.setPower(power);
    }

    public boolean isHomePressed() {
        return turretLimitSwitch.isPressed();
    }

    public void resetEncoderWhenHomePressed() {
        boolean pressed = turretLimitSwitch.isPressed();

        if (pressed && !wasPressed) {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        wasPressed = pressed;
    }

    public double autoAim(double robotX, double robotY, double robotHeadingDeg) {
        double dx = GOAL_X - robotX;
        double dy = GOAL_Y - robotY;

        double fieldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));

        double turretAngle = fieldAngleToGoal - robotHeadingDeg + 180;

        return normalizeDegrees(turretAngle);
    }

    public double normalizeDegrees(double angle) {
        // keep the angle between -180 and 180 degrees
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    public void manual(double power) {
        turret.setPower(power);
    }
}