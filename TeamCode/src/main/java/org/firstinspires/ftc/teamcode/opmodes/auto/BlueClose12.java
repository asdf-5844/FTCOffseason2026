package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierLine;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "BlueClose12")
public class BlueClose12 extends LinearOpMode{
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;

    private static final double TRAVEL_POWER = 0.80;  // normal driving

    private final Pose startPose = new Pose(28.3154, 131.7978, Math.toRadians(145));
    private final Pose scorePose = new Pose(61.2863, 84, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(17, 84, Math.toRadians(180));
    private final Pose gatePose = new Pose(16.9621, 75.5462555, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(11, 60, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(11, 36, Math.toRadians(180));
    private final Pose endPose = new Pose (25.788, 64.652);

    private PathChain scorePreload, grabPickup1, openGate, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, leave;

    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, gatePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(64, 60), pickup2Pose))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, new Pose(50, 61.58), scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(68.535, 36), pickup3Pose))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public Command intakeOn() {
        return Command.build()
                .setStart(() -> intake.intake())
                .setDone(() -> true);
    }

    public Command intakeOff() {
        return Command.build()
                .setStart(() -> intake.stop())
                .setDone(() -> true);
    }

    public Command spinUpShooter() {
        return Command.build()
                .setStart(() -> {
                    turret.setTargetAngle(Math.toDegrees(Math.atan2(60, -61.2863)));

                    shooter.aimForDistance(85.76);
                    shooter.requestSpinUp(shooter.getFlywheelSpeed(85.76));
                 })
                .setDone(() -> shooter.getState() == Shooter.ShooterState.READY);
    }

    public Command feed() {
        return Command.build()
                .setStart(() -> shooter.requestFeed())
                .setDone(() ->
                        shooter.getState() != Shooter.ShooterState.FEEDING
                );
    }

    public Command autoRoutine() {
        return sequential(
                // PRELOAD
                spinUpShooter(),
                follow(follower, scorePreload),
                feed(),

                // FIRST 3
                intakeOn(),
                follow(follower, grabPickup1, true),
                intakeOff(),
                follow(follower, openGate, true),
                follow(follower, scorePickup1, true),
                feed(),

                // SECOND 3
                intakeOn(),
                follow(follower, grabPickup2, true),
                intakeOff(),
                follow(follower, scorePickup2, true),
                feed(),

                // THIRD 3
                intakeOn(),
                follow(follower, grabPickup3, true),
                intakeOff(),
                follow(follower, scorePickup3, true),
                feed(),

                // LEAVE
                follow(follower, leave, true)
        );
    }

    @Override
    public void runOpMode() {
        // These will run when the OpMode is initiated
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);

        buildPaths();

        follower.setStartingPose(startPose);
        follower.setMaxPower(TRAVEL_POWER);

        waitForStart();
        // We schedule all our commands when we start the OpMode
        schedule(autoRoutine());
        while (opModeIsActive()) {
            // Update the follower and execute the scheduler every loop
            follower.update();
            Scheduler.execute();

            shooter.update();
            turret.update();

            // Feedback to Driver Hub for debugging
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}
