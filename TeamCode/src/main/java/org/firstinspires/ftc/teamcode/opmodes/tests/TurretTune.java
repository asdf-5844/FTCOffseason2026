package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "TurretTune", group = "Test")
public class TurretTune extends LinearOpMode {
    private Turret turret;

    @Override
    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad2.a) {
                turret.setTargetAngle(0);
            }

            if (gamepad2.b) {
                turret.setTargetAngle(30);
            }

            if (gamepad2.x) {
                turret.setTargetAngle(-30);
            }

            if (gamepad2.y) {
                turret.setTargetAngle(60);
            }

            // turret.resetEncoderWhenHomePressed();
            // turret.update();

            telemetry.addData("Error", turret.getError());
            telemetry.addData("Angle", turret.getCurrentAngle());
            telemetry.addData("Pressed", turret.isHomePressed());
            telemetry.update();
        }
    }
}
