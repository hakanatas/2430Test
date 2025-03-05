package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.List;

@Autonomous(name = "aa Skibbidi Sigma Six Spec", preselectTeleOp = "aaTeleop")
public class sketchSixSpec extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private EndEffector endEffector;
    private Deposit deposit;
    private int pathState;
    private int specCounter = 0;

    private double power;

    private static final double SCORE_VEL_THRESHOLD = 0.8;
    private static final double INTAKE_VEL_THRESHOLD = 0.5;

    private static final int SLIDE_SCORE = 450;
    private static final int SLIDE_SAFE = 350;

    private static final Pose sixSpecLocation = new Pose(0,0,180);

    private PathChain preload,intakeSixSpec, combinedPush, score, intake;

    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(newStartPoint(), pointFromPose(sixSpecPoses.preloadPose)))
                .setConstantHeadingInterpolation(180)
                .build();

        intakeSixSpec = follower.pathBuilder()
                .addPath(new BezierLine(pointFromPose(sixSpecPoses.preloadPose),pointFromPose(sixSpecLocation)))
                .setConstantHeadingInterpolation(180)
                .build();

        combinedPush = follower.pathBuilder()
                .addPath(new BezierCurve(pointFromPose(sixSpecLocation), pointFromPose(sixSpecPoses.sixSpecDropOffControl),pointFromPose(sixSpecPoses.sixSpecDropOffEnd)))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(225))
                .addPath(new BezierCurve(pointFromPose(sixSpecPoses.sixSpecDropOffEnd), pointFromPose(sixSpecPoses.spline1Control), pointFromPose(sixSpecPoses.spline1End)))
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .addPath(new BezierCurve(pointFromPose(sixSpecPoses.spline1End), pointFromPose(sixSpecPoses.spline2Control), pointFromPose(sixSpecPoses.spline2End)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(pointFromPose(sixSpecPoses.spline2End), pointFromPose(sixSpecPoses.spline3Control), pointFromPose(sixSpecPoses.spline3End)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(pointFromPose(sixSpecPoses.spline3End), pointFromPose(sixSpecPoses.pushSample2End)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(pointFromPose(sixSpecPoses.pushSample2End), pointFromPose(sixSpecPoses.spline4Control), pointFromPose(sixSpecPoses.spline4End)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(pointFromPose(sixSpecPoses.spline4End), pointFromPose(sixSpecPoses.pushSample3End)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(pointFromPose(sixSpecPoses.pushSample3End), pointFromPose(sixSpecPoses.spline5Control), new Point(sixSpecPoses.spline5End.getX() - 1, sixSpecPoses.spline5End.getY() + 2)))
                .setConstantHeadingInterpolation(0)
                .build();

        score = follower.pathBuilder()
                .addPath(new BezierLine(pointFromPose(sixSpecPoses.spline5End), pointFromPose(sixSpecPoses.scoreEnd)))
                .setConstantHeadingInterpolation(0)
                .build();

        intake = follower.pathBuilder()
                .addPath(new BezierLine(pointFromPose(sixSpecPoses.scoreEnd), pointFromPose(sixSpecPoses.spline5End)))
                .setConstantHeadingInterpolation(0)
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
                preScore();
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(combinedPush, true);
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
                    if (specCounter == 5) {
                        setPathState(-1);
                    } else {
                        setPathState(3);
                    }
                }
                break;
            default:
                if (follower.getPose().getY() < 65 && follower.getPose().getX() < 50 && specCounter == 5) {
                    requestOpModeStop();
                }
                intakePrep();
        }
    }

    public void score() {
        if (follower.getPose().getX() > 10 && follower.getPose().getX() < 20 && follower.isBusy()) {
            endEffector.setSpecScore();
            deposit.setSlideTarget(SLIDE_SCORE);
        }

        if (follower.getVelocityMagnitude() < SCORE_VEL_THRESHOLD && follower.getPose().getX() > 35) {
            endEffector.openClaw();
            deposit.setSlideTarget(SLIDE_SAFE);
            specCounter++;
            follower.breakFollowing();
            setPathState();
        }
    }
    public void preScore() {
        if (follower.getPose().getX() > 5 && follower.getPose().getX() < 20 && follower.isBusy()) {
            endEffector.setSpecScore();
            deposit.setSlideTarget(SLIDE_SCORE);
        }

        if (follower.getVelocityMagnitude() < SCORE_VEL_THRESHOLD && follower.getPose().getX() > 35) {
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
                if (specCounter >= 5) {
                    endEffector.setSpecScore();
                } else {
                    endEffector.setWallIntakePositionAlt();
                }
            }
            if (specCounter >= 5) {
                setPathState(-1);
            } else {
                setPathState();
            }
        }
    }

    public void intake() {
        if (follower.getVelocityMagnitude() < INTAKE_VEL_THRESHOLD && follower.getPose().getX() < 10 && endEffector.clawPosition != 0.13) {
            endEffector.closeClaw();
            pathTimer.resetTimer();
        }

        if (endEffector.clawPosition == 0.13 && pathTimer.getElapsedTime() > 200) {
            endEffector.setSpecScore();
            setPathState();
        }
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
        telemetry.addData("Specs Scored", specCounter);
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Path Busy", follower.isBusy());
        telemetry.addData("Power", power);


        telemetry.update();

        if (specCounter == 5) {
            follower.setMaxPower(1);
            power = 1;
        } else if (follower.getPose().getX() < 40 && follower.getPose().getY() < 60 && follower.getPose().getY() > 28 && follower.getVelocity().getXComponent() < 0 && specCounter != 1) {
            follower.setMaxPower(0.35);
            power = 0.35;
        } else if (follower.getPose().getX() < 24 && follower.getPose().getY() < 48 && follower.getPose().getY() > 24 && follower.getPose().getY() > 10 && follower.getVelocity().getXComponent() < 0) {
            follower.setMaxPower(0.35);
            power = 0.35;
        } else if (specCounter == 1) {
            follower.setMaxPower(1);
            power = 1;
        }else {
            follower.setMaxPower(1);
            power = 1;
        }

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

        deposit = new Deposit(hardwareMap, telemetry, true);
        endEffector = new EndEffector(hardwareMap);
        endEffector.setLight(0.5);
        endEffector.override = false;

        endEffector.setAutoIdle();
        deposit.setPivotTarget(121);
        deposit.setSlideTarget(0);

        // Build our newly incorporated multi-step path:
        //buildPaths();

        if (!deposit.slideLimit.isPressed()) {
            throw new IllegalArgumentException("Zero slides before init");
        }
        telemetry.addLine("Good To Move Robot");
    }

    @Override
    public void init_loop() {
        endEffector.setAutoIdle();
        follower.getPose();
        telemetry.addData("Pose:",follower.getPose());
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        follower.getPose();
        //newStartPoint();
        buildPaths();
        setPathState(0);
    }

    private Point pointFromPose(Pose pose) {
        // Replace getX() and getY() with your actual accessor methods if needed.
        return new Point(pose.getX(), pose.getY(), Point.CARTESIAN);
    }

    private Point newStartPoint(){
        return new Point(follower.getPose().getX(),follower.getPose().getY(),Point.CARTESIAN);
    }

    public void findSixSpec(){

    }

}
