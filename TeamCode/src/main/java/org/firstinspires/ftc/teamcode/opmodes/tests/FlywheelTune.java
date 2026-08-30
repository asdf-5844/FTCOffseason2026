package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "New Custom Flywheel Tuner", group = "Test")
public class FlywheelTune extends OpMode {

    public DcMotorEx flywheelMotorLeft;
    public DcMotorEx flywheelMotorRight;

    public static double targetVelocity = 0;

    public static double kS = 0.0; // static friction compensation
    public static double kV = 0.0; // velocity feedforward
    public static double kP = 0.0; // proportional feedback

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flywheelMotorLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelMotorRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");

        flywheelMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        double currentVelocity = flywheelMotorLeft.getVelocity();
        double error = targetVelocity - currentVelocity;

        double output = 0;

        output += kS * Math.signum(targetVelocity);
        output += kV * targetVelocity;
        output += kP * error;

        output = Range.clip(output, 0.0, 1.0);

        flywheelMotorLeft.setPower(output);
        flywheelMotorRight.setPower(output);

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("Output", output);
        telemetry.addData("kS Output", kS * Math.signum(targetVelocity));
        telemetry.addData("kV Output", kV * targetVelocity);
        telemetry.addData("kP Output", kP * error);
        telemetry.update();
    }
}