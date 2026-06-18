package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.subsystems.Intake;

// minimal OpMode skeleton

/*
another way to get OpMode boilerplate code
is to hover over the red squiggle under:
"public class [name] extends LinearOpMode"
and click implement methods
*/
@Disabled
@TeleOp
public class BareOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        Intake intake = new Intake(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            intake.intake.setPower(power);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}