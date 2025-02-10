package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.List;

import org.firstinspires.ftc.teamcode.config.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.opmode.Auto.Poses.*;

@Autonomous(name = "God Five Spec", group = "auto", preselectTeleOp = "Teleop")
public class godFiveSpec extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private EndEffector endEffector;
    private Deposit deposit;
    private int pathState;
    private int specCounter = 0;

    private static final double VEL_THRESHOLD = 0.5;
    private static final int SLIDE_SCORE = 450;
    private static final int SLIDE_SAFE = 300;

    private PathChain preload, combinedPush, score, intake;

    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(pointFromPose(Poses.startPose), pointFromPose(Poses.preloadPose)))
                .setConstantHeadingInterpolation(0)
                .build();

        combinedPush = follower.pathBuilder()
                .addPath(new BezierCurve(pointFromPose(Poses.preloadPose), pointFromPose(Poses.spline1Control), pointFromPose(Poses.spline1End)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(pointFromPose(Poses.spline1End), pointFromPose(Poses.spline2Control), pointFromPose(Poses.spline2End)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(pointFromPose(Poses.spline2End), pointFromPose(Poses.spline3Control), pointFromPose(Poses.spline3End)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(pointFromPose(Poses.spline3End), pointFromPose(Poses.pushSample2End)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(pointFromPose(Poses.pushSample2End), pointFromPose(Poses.spline4Control), pointFromPose(Poses.spline4End)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(pointFromPose(Poses.spline4End), pointFromPose(Poses.pushSample3End)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(pointFromPose(Poses.pushSample3End), pointFromPose(Poses.spline5Control), pointFromPose(Poses.spline5End)))
                .setConstantHeadingInterpolation(0)
                .addParametricCallback(0.9, () -> follower.setMaxPower(0.85))
                .build();

        score = follower.pathBuilder()
                .addPath(new BezierLine(pointFromPose(Poses.spline5End), pointFromPose(Poses.scoreEnd)))
                .setConstantHeadingInterpolation(0)
                .build();

        intake = follower.pathBuilder()
                .addPath(new BezierLine(pointFromPose(Poses.scoreEnd), pointFromPose(Poses.spline5End)))
                .setConstantHeadingInterpolation(0)
                .addParametricCallback(0.9, () -> follower.setMaxPower(0.85))
                .build();
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(preload, true);
                    setPathState();
                }
                break;
            case 1:
                score();
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(combinedPush, 1.0, true);
                    setPathState();
                }
                break;
            case 3:
                intakePrep();
                break;
            case 4:
                intake();
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(score, true);
                    setPathState();
                }
                break;
            case 6:
                score();
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(intake, true);
                    setPathState();
                }
                break;
            case 8:
                intakePrep();
                break;
            case 9:
                intake();
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(score, true);
                    setPathState();
                }
                break;
            case 11:
                score();
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(intake, true);
                    setPathState();
                }
                break;
            case 13:
                intakePrep();
                break;
            case 14:
                intake();
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(score, true);
                    setPathState();
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(intake, true);
                    setPathState();
                }
                break;
            case 17:
                intakePrep();
                break;
            case 18:
                intake();
                break;
            case 19:
                if (!follower.isBusy()) {
                    follower.followPath(score, true);
                    setPathState();
                }
                break;
            case 20:
                if (!follower.isBusy()) {
                    follower.followPath(intake, true);
                    setPathState();
                }
                break;
            default:
                endEffector.openClaw();
        }
    }

    public void score() {
        if (follower.getPose().getX() > 10 && follower.getPose().getX() < 20 && follower.isBusy()) {
            endEffector.setSpecScore();
            deposit.setSlideTarget(SLIDE_SCORE);
        }

        if (follower.getVelocityMagnitude() < VEL_THRESHOLD && follower.getPose().getX() > 35) {
            endEffector.openClaw();
            deposit.setSlideTarget(SLIDE_SAFE);
            specCounter++;
            follower.breakFollowing();
            setPathState();
        }
    }

    public void intakePrep() {
        if (pathTimer.getElapsedTimeSeconds() > 0.25) {
            deposit.setSlideTarget(0);
            if (deposit.slidesReached) {
                endEffector.setWallIntakePositionAlt();
            }
            setPathState();
        }
    }

    public void intake() {
        if (follower.getVelocityMagnitude() < VEL_THRESHOLD && follower.getPose().getX() < 9) {
            endEffector.closeClaw();
            if (pathTimer.getElapsedTime() > 300) {
                endEffector.setSpecScore();
                setPathState();
            }
        }
        pathTimer.resetTimer();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setPathState() {
        pathState += 1;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("X Pos", follower.getPose().getX());
        telemetry.addData("Y Pos", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Path Busy", follower.isBusy());


        telemetry.update();
        follower.setMaxPower(0.9);



        deposit.update();

        autonomousPathUpdate();
    }

    @Override
    public void stop() {
        endEffector.openClaw();
    }

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        pathTimer = new Timer();

        // Initialize follower, slides, etc. as usual
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        // If needed, set a starting pose (only if your system requires it)
        // follower.setStartingPose(new Pose(0,0, 0));
        follower.setStartingPose(Poses.startPose);
        follower.setMaxPower(0.85);

        deposit = new Deposit(hardwareMap, telemetry, true);
        endEffector = new EndEffector(hardwareMap);
        endEffector.setLight(0.5);
        endEffector.override = false;

        endEffector.init();
        deposit.setPivotTarget(121);
        deposit.setSlideTarget(0);

        // Build our newly incorporated multi-step path:
        buildPaths();

//        if (!deposit.slideLimit.isPressed()) {
//            throw new IllegalArgumentException("Zero slides before init");
//        }
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    private Point pointFromPose(Pose pose) {
        // Replace getX() and getY() with your actual accessor methods if needed.
        return new Point(pose.getX(), pose.getY(), Point.CARTESIAN);
    }


}
