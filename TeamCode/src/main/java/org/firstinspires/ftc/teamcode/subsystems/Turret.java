package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.util.GlobalConstants;

@Config
public class Turret {
    private final DcMotorEx turret;
    private final TouchSensor turretLimitSwitch; // Magnetic Limit Switch
    private boolean wasPressed = false;

    private double targetAngle = 0;

    // Tune PID!!!
    public static double kP = 0.018;
    public static double kD = 0.0008;
    public static double kS = 0.025; // friction
    public static double DEADBAND_DEG = 1.2;
    private double previousError = 0;
    private long previousTimeNanos = 0;

    public static final double GOAL_X = GlobalConstants.GOAL_X;
    public static final double GOAL_Y = GlobalConstants.GOAL_Y;
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
        return normalizeDegrees(targetAngle - getCurrentAngle());
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
        double error = getError();

        long currentTimeNanos = System.nanoTime();
        // on the first loop, there is no previous time or error yet
        if (previousTimeNanos == 0) {
            previousTimeNanos = currentTimeNanos;
            previousError = error;
            turret.setPower(0);
            return;
        }

        // there are one billion nanoseconds in one second
        double deltaTime =
                (currentTimeNanos - previousTimeNanos)
                        / 1_000_000_000.0;

        if (deltaTime <= 0) {
            return;
        }

        double proportional = kP * error;

        double derivative =
                (error - previousError) / deltaTime;
        double derivativePower = kD * derivative;

        // P + D
        double power = proportional + derivativePower;

        if (Math.abs(error) > DEADBAND_DEG) {
            // Small constant push to overcome friction
            power += Math.signum(error) * kS;
        } else {
            power = 0;
        }

        // clamp, speed limits
        if (power > 0.8) {
            power = 0.8;
        }
        if (power < -0.8) {
            power = -0.8;
        }

        turret.setPower(power);

        // Save new values for the next loop.
        previousError = error;
        previousTimeNanos = currentTimeNanos;
    }

    public boolean isHomePressed() {
        return turretLimitSwitch.isPressed();
    }

    public void resetEncoderWhenHomePressed() {
        boolean pressed = turretLimitSwitch.isPressed();

        if (pressed && !wasPressed) {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            targetAngle = 0;
            previousError = 0;
            previousTimeNanos = 0;
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