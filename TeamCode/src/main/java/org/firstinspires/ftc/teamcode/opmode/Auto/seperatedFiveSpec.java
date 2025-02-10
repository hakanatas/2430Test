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


@Autonomous(name = "New Five Spec", group = "auto", preselectTeleOp = "Teleop")
public class seperatedFiveSpec extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private EndEffector endEffector;
    private Deposit deposit;
    private int pathState;

    private PathChain preload, combinedPush, score1, return1, score2, return2, score3, return3, score4, return4, inter, line, reverseLine, repickup;
    private final Pose startingPose =  new Pose   (7, 65.859 - 12, Math.toRadians(0));

    private double wall_intake = 7;
    public double wall_offset = 1.2;
    private double subX = 39.7;
    private double startY = 65.859;
    private double yInc = 2;
    private double splineControl = 22;

    private int slideScorePos = 400;

    /**
     * Build our complex path sequence using the same calls
     * that were in the GeneratedPath constructor.
     */
    public void buildPaths() {
        // Preload path (Line 1)
        preload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(new Pose(7.338, 65.859 - 12, Math.toRadians(0))),
                        new Point(new Pose(40, 65.859, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        combinedPush = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(40.000, 65.859, Math.toRadians(0))),
                        new Point(new Pose(34.000, 67.000, Math.toRadians(0))),
                        new Point(new Pose(0.000, 48.000, Math.toRadians(0))),
                        new Point(new Pose(50, 30.000, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(50, 30.000, Math.toRadians(0))),
                        new Point(new Pose(65.000, 20.000, Math.toRadians(0))),
                        new Point(new Pose(28, 20.000, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(28, 20.000, Math.toRadians(0))),
                        new Point(new Pose(65.000, 28, Math.toRadians(0))),
                        new Point(new Pose(50, 15, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(new Pose(50, 15, Math.toRadians(0))),
                        new Point(new Pose(28, 15, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(28, 15, Math.toRadians(0))),
                        new Point(new Pose(65.000, 16.250, Math.toRadians(0))),
                        new Point(new Pose(50, 7.9, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(50.000, 7.9, Point.CARTESIAN),
                                new Point(19, 7.9, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(19, 7.5, Point.CARTESIAN),
                                new Point(19, 30, Point.CARTESIAN),
                                new Point(wall_intake, 30, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        inter = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(19, 7.5, Point.CARTESIAN),
                                new Point(19, 30, Point.CARTESIAN),
                                new Point(wall_intake, 30, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(new Pose(wall_intake, 30, Math.toRadians(0))),
                        new Point(new Pose(subX, calcY(1), Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        return1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(subX, calcY(1), Point.CARTESIAN),
                                new Point(wall_intake + 10, 30, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(wall_intake + 10, 30, Point.CARTESIAN),
                                new Point(wall_intake + wall_offset, 30, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        repickup = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(wall_intake+wall_offset, 30, Point.CARTESIAN),
                                new Point(wall_intake + 10, 30, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(wall_intake + 10, 30, Point.CARTESIAN),
                                new Point(wall_intake+wall_offset, 30, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        score2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(wall_intake + wall_offset, 30, Point.CARTESIAN),
                                new Point(subX, calcY(2), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(subX, 68, Point.CARTESIAN), new Point(subX, 68.5, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        return2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(subX, calcY(2), Point.CARTESIAN),
                                new Point(wall_intake + 10, 30, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        score3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(wall_intake + wall_offset, 30, Point.CARTESIAN),

                                new Point(subX, calcY(3), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(subX, 68, Point.CARTESIAN), new Point(subX, 68.5, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        return3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(subX, calcY(3), Point.CARTESIAN),

                                new Point(wall_intake + 10, 30, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        score4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(wall_intake + wall_offset, 30, Point.CARTESIAN),

                                new Point(subX, calcY(4), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(subX, 68, Point.CARTESIAN), new Point(subX, 68.5, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    // Start following our newly built path
                    follower.followPath(preload, 1, false);
                    endEffector.setSpecScore();
                    setPathState();

                }
                break;
            case 1:
                if (follower.getPose().getX() > 10 && follower.isBusy()) {
                    deposit.setSlideTarget(slideScorePos);
                }

                if (!follower.isBusy() && follower.getPose().getX() >= 19 && follower.getVelocityMagnitude() < 0.5) {
                    endEffector.openClaw();
                }
                break;
            case 2:
                if (!follower.isBusy() || deposit.slidesReached) {
                    deposit.setSlideTarget(0);
                    endEffector.setWallIntakePositionAlt();
                    follower.followPath(combinedPush, 1,false);
                    setPathState(4);
                }
                break;
//            case 3:
//                if(!follower.isBusy()) {
//                    follower.followPath(inter,0.7, false);
//                    setPathState();
//                }
//                break;
            case 4:
                if ((!follower.isBusy() || follower.getPose().getX() < 8) && follower.getVelocityMagnitude() < 0.5 && pathTimer.getElapsedTimeSeconds() > 0.5 && deposit.slidesReached) {
                    endEffector.closeClaw();
                    endEffector.setSpecScore();
                    setPathState();
                }
                break;
            case 5:
                if ((!follower.isBusy() || follower.getPose().getX() < 8) && follower.getVelocityMagnitude() < 0.5 && pathTimer.getElapsedTimeSeconds() > 0.5 && deposit.slidesReached) {
                    follower.followPath(score1, false);
                    setPathState();
                }
                break;
            case 6:
                if (follower.getPose().getY() > 40 && follower.isBusy()) {
                    deposit.setSlideTarget(slideScorePos);
                }
                if (follower.getPose().getX() >= 19 && !follower.isBusy() && follower.getVelocityMagnitude() < 0.5) {
                    endEffector.openClaw();
                }
                break;
            case 7:
                if(!follower.isBusy() || deposit.slidesReached) {
                    deposit.setSlideTarget(0);
                    endEffector.setWallIntakePositionAlt();
                    follower.followPath(return1,1, false);
                    setPathState();
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(line,0.6, false);
                    setPathState();
                }
                break;
            case 9:
                // Return 1
                if (!follower.isBusy()) {
                    endEffector.closeClaw();
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        deposit.setSlideTarget(215);
                        setPathState();
                    }
                } else {
                    if (!follower.isBusy() || follower.getVelocityMagnitude() < 0.8 && !endEffector.either()) {
                        follower.followPath(repickup, 0.5, false);
                    }
                    pathTimer.resetTimer();
                }
                break;
            case 10:
                if ((!follower.isBusy() || follower.getPose().getX() < 8) && follower.getVelocityMagnitude() < 0.5 && pathTimer.getElapsedTimeSeconds() > 0.5 && deposit.slidesReached) {
                    endEffector.closeClaw();
                    follower.followPath(score2, false);
                    setPathState();
                }
                break;
            case 11:
                if (follower.getPose().getX() > 15 && follower.isBusy()) {
                    deposit.setSlideTarget(120);
                    endEffector.setSpecScore(); // Adjust claw for scoring
                    // Set initial slide position
                }

                if (follower.getPose().getY() > 40 && follower.isBusy()) {
                    deposit.setSlideTarget(460);
                }


                if (follower.getPose().getX() >= 19 && !follower.isBusy() && follower.getVelocityMagnitude() < 0.5) {
                    deposit.setSlideTarget(230);
                    if (deposit.liftPos < 250) {
                        endEffector.openClaw();
                        setPathState();
                    }
                }
                break;
            case 12:
                if(!follower.isBusy() || deposit.slidesReached) {
                    endEffector.openClaw();
                    deposit.setSlideTarget(0);
                    deposit.setPivotTarget(121);
                    endEffector.setWallIntakePositionAlt();
                    follower.followPath(return2,1, false);
                    setPathState();
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    follower.followPath(line,0.6, false);
                    setPathState();
                }
                break;
            case 14:
                // Return 1
                if (!follower.isBusy()) {
                    endEffector.closeClaw();
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        deposit.setSlideTarget(215);
                        setPathState();
                    }
                }
                break;
            case 15:
                if ((!follower.isBusy() || follower.getPose().getX() < 8) && follower.getVelocityMagnitude() < 0.5 && pathTimer.getElapsedTimeSeconds() > 0.5 && deposit.slidesReached) {
                    endEffector.closeClaw();
                    follower.followPath(score3, false);
                    setPathState();
                }
                break;
            case 16:
                if (follower.getPose().getX() > 15 && follower.isBusy()) {
                    deposit.setSlideTarget(120);
                    endEffector.setSpecScore(); // Adjust claw for scoring
                    // Set initial slide position
                }

                if (follower.getPose().getY() > 40 && follower.isBusy()) {
                    deposit.setSlideTarget(460);
                }


                if (follower.getPose().getX() >= 19 && !follower.isBusy() && follower.getVelocityMagnitude() < 0.5) {
                    deposit.setSlideTarget(230);
                    if (deposit.liftPos < 250) {
                        endEffector.openClaw();
                        setPathState();
                    }
                }
                break;
            case 17:
                if(!follower.isBusy() || deposit.slidesReached) {
                    endEffector.openClaw();
                    deposit.setSlideTarget(0);
                    deposit.setPivotTarget(121);
                    endEffector.setWallIntakePositionAlt();
                    follower.followPath(return3,1, false);
                    setPathState();
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    follower.followPath(line,1, false);
                    setPathState();
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    endEffector.closeClaw();
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        deposit.setSlideTarget(215);
                        setPathState();
                    }
                } else {
                    if (!follower.isBusy() || follower.getVelocityMagnitude() < 0.8 && !endEffector.either()) {
                        follower.followPath(repickup, 0.5, false);
                    }
                    pathTimer.resetTimer();
                }
                break;
            case 20:
                if ((!follower.isBusy() || follower.getPose().getX() < 8) && follower.getVelocityMagnitude() < 0.5 && pathTimer.getElapsedTimeSeconds() > 0.5 && deposit.slidesReached) {
                    endEffector.closeClaw();
                    follower.followPath(score4, false);
                    setPathState();
                }
                break;
            case 21:
                if (follower.getPose().getX() > 15 && follower.isBusy()) {
                    deposit.setSlideTarget(120);
                    endEffector.setSpecScore(); // Adjust claw for scoring
                    // Set initial slide position
                }

                if (follower.getPose().getY() > 40 && follower.isBusy()) {
                    deposit.setSlideTarget(460);
                }


                if (follower.getPose().getX() >= 19 && !follower.isBusy() && follower.getVelocityMagnitude() < 0.5) {
                    deposit.setSlideTarget(230);
                    if (deposit.liftPos < 250) {
                        endEffector.openClaw();
                        setPathState();
                    }
                }
                break;
            default:
                if (!follower.isBusy()) {
                    endEffector.openClaw();
                    requestOpModeStop();
                }

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
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Path Busy", follower.isBusy());


        telemetry.update();



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
        follower.setStartingPose(startingPose);
        follower.setMaxPower(1);

        deposit = new Deposit(hardwareMap, telemetry, true);
        endEffector = new EndEffector(hardwareMap);
        endEffector.setLight(0.5);
        endEffector.override = false;

        endEffector.init();
        deposit.setPivotTarget(121);
        deposit.setSlideTarget(50);

        // Build our newly incorporated multi-step path:
        buildPaths();

        if (!deposit.slideLimit.isPressed()) {
            throw new IllegalArgumentException("Zero slides before init");
        }
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    private double calcY(double n) {
        return startY + n * yInc;
    }

}