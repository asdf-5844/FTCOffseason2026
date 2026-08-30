package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueClose12")
public class BlueClose12 {
    private Follower follower;

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
                .setConstantHeadingInterpolation(180)
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, gatePose))
                .setConstantHeadingInterpolation(180)
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, scorePose))
                .setConstantHeadingInterpolation(180)
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(64, 60), pickup2Pose))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, new Pose(50, 61.58), scorePose))
                .setConstantHeadingInterpolation(180)
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(68.535, 36), pickup3Pose))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setConstantHeadingInterpolation(180)
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setConstantHeadingInterpolation(180)
                .build();
    }


}
