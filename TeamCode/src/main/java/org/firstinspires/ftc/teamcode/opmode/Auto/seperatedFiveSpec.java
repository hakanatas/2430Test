package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.lynx.LynxModule;

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

    private PathChain preload, combinedPush, score1, return1, score2, return2, score3, return3, score4, return4, inter;
    private final Pose startingPose =  new Pose   (7, 65.859, Math.toRadians(0));

    private double wall_intake = 7.5;
    private double subX = 39;
    private double startY = 65.859;
    private double yInc = 3;


    /**
     * Build our complex path sequence using the same calls
     * that were in the GeneratedPath constructor.
     */
    public void buildPaths() {
        // Preload path (Line 1)
        preload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(new Pose(7.338, 65.859, Math.toRadians(0))),
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
                        new Point(new Pose(22, 20.000, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(22, 20.000, Math.toRadians(0))),
                        new Point(new Pose(65.000, 28, Math.toRadians(0))),
                        new Point(new Pose(50, 13.5, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(new Pose(50, 13.5, Math.toRadians(0))),
                        new Point(new Pose(22, 13.5, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(22, 13.5, Math.toRadians(0))),
                        new Point(new Pose(65.000, 16.250, Math.toRadians(0))),
                        new Point(new Pose(50, 7.5, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(50.000, 7.5, Point.CARTESIAN),
                                new Point(19, 7.5, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        inter = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(19, 7.5, Point.CARTESIAN),
                                new Point(17.3, 13, Point.CARTESIAN),
                                new Point(wall_intake, 28, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(wall_intake, 28, Math.toRadians(0))),
                        new Point(new Pose(subX, 28, Math.toRadians(0))),
                        new Point(new Pose(30, calcY(1), Math.toRadians(0))),
                        new Point(new Pose(subX, calcY(1), Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        return1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(subX, calcY(1), Point.CARTESIAN),
                                new Point(30, calcY(1), Point.CARTESIAN),
                                new Point(subX, 28, Point.CARTESIAN),
                                new Point(wall_intake + 5, 28.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(wall_intake + 5, 28.000, Point.CARTESIAN),
                                new Point(wall_intake, 28.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        score2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(wall_intake, 28.000, Point.CARTESIAN),
                                new Point(subX, 28, Point.CARTESIAN),
                                new Point(30, calcY(2), Point.CARTESIAN),
                                new Point(subX, calcY(2), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(subX, 68, Point.CARTESIAN), new Point(subX, 68.5, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        return2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(subX, calcY(2), Point.CARTESIAN),
                                new Point(30, calcY(2), Point.CARTESIAN),
                                new Point(subX, 28, Point.CARTESIAN),
                                new Point(wall_intake, 28.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(wall_intake + 5, 28.000, Point.CARTESIAN),
                                new Point(wall_intake, 28.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        score3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(wall_intake, 28.000, Point.CARTESIAN),
                                new Point(subX, 28, Point.CARTESIAN),
                                new Point(30, calcY(3), Point.CARTESIAN),
                                new Point(subX, calcY(3), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(subX, 68, Point.CARTESIAN), new Point(subX, 68.5, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        return3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(subX, calcY(3), Point.CARTESIAN),
                                new Point(30, calcY(3), Point.CARTESIAN),
                                new Point(subX, 28, Point.CARTESIAN),
                                new Point(wall_intake, 28.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(wall_intake + 5, 28.000, Point.CARTESIAN),
                                new Point(wall_intake, 28.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        score4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(wall_intake, 28.000, Point.CARTESIAN),
                                new Point(subX, 28, Point.CARTESIAN),
                                new Point(30, calcY(4), Point.CARTESIAN),
                                new Point(subX, calcY(4), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(subX, 68, Point.CARTESIAN), new Point(subX, 68.5, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        return4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(subX, calcY(4), Point.CARTESIAN),
                                new Point(30, calcY(4), Point.CARTESIAN),
                                new Point(subX, 28, Point.CARTESIAN),
                                new Point(wall_intake, 28.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(wall_intake + 5, 28.000, Point.CARTESIAN),
                                new Point(wall_intake, 28.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    // Start following our newly built path
                    follower.followPath(preload, 1, true);
                    endEffector.setSpecScore();

                }
                if (pathTimer.getElapsedTimeSeconds() > 0.2 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                    deposit.setSlideTarget(500);
                }

                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    deposit.setSlideTarget(230);
                    if (deposit.liftPos < 250) {
                        endEffector.openClaw();
                        setPathState(pathState + 1);
                    }
                }
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 5) {
                    deposit.setSlideTarget(0);
                    deposit.setPivotTarget(90);
                    endEffector.setWallIntakePositionAlt();
                    follower.followPath(combinedPush, 1,false);
                    setPathState();
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(inter,0.6, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 1.2) {
                    endEffector.closeClaw();
                    deposit.setSlideTarget(50);
                    setPathState(5);
                }
                break;
            case 5:
                cycle(1);
                break;

            case 6:
                intake(1);
                break;
            case 7:
                cycle(2);
                break;
            case 8:
                intake(2);
                break;
            case 9:
                cycle(3);
                break;
            case 10:
                intake(3);
                break;
            case 11:
                cycle(4);
                if (pathTimer.getElapsedTimeSeconds() > 4.4) {
                    deposit.setSlideTarget(230);
                    if (deposit.liftPos < 250) {
                        endEffector.openClaw();
                        setPathState(pathState + 1); // Transition to next return
                    }
                }
                break;
            case 12:
                setPathState(-1);
                break;
            default:
                if (!follower.isBusy()) {
                    requestOpModeStop(); // Stop the OpMode if all states are complete
                }
                break;
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

    private void cycle(int n) {
        switch (n) {
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(score1, true); // Follow score1 path
                }

                if (pathTimer.getElapsedTimeSeconds() > 0.3 && pathTimer.getElapsedTimeSeconds() < 1) {
                    deposit.setSlideTarget(110);
                    endEffector.setSpecScore(); // Adjust claw for scoring
                    // Set initial slide position
                }


                if (pathTimer.getElapsedTimeSeconds() > 1.2 && pathTimer.getElapsedTimeSeconds() < 2.5) {
                    deposit.setSlideTarget(460); // Lift slide to scoring position
                }

                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    deposit.setSlideTarget(230); // Lower slide slightly for object release
                    if (deposit.liftPos < 250) {
                        endEffector.openClaw(); // Release object
                        setPathState(pathState + 1); // Transition to return state
                    }
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(score2, true); // Follow score2 path
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.8 && pathTimer.getElapsedTimeSeconds() < 2.3) {
                    endEffector.closeClaw();
                    deposit.setSlideTarget(110);
                }

                if (pathTimer.getElapsedTimeSeconds() > 2.3 && pathTimer.getElapsedTimeSeconds() < 3) {
                    endEffector.setSpecScore();
                    deposit.setSlideTarget(100);
                }

                if (pathTimer.getElapsedTimeSeconds() > 3.2 && pathTimer.getElapsedTimeSeconds() < 4.3) {
                    deposit.setSlideTarget(460);
                }

                if (pathTimer.getElapsedTimeSeconds() > 4.6) {
                    deposit.setSlideTarget(230);
                    if (deposit.liftPos < 250) {
                        endEffector.openClaw();
                        setPathState(pathState + 1); // Transition to next return
                    }
                }
                break;
            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(score3, true); // Follow score2 path
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.8 && pathTimer.getElapsedTimeSeconds() < 2.3) {
                    endEffector.closeClaw();
                    deposit.setSlideTarget(110);
                }

                if (pathTimer.getElapsedTimeSeconds() > 2.3 && pathTimer.getElapsedTimeSeconds() < 3) {
                    endEffector.setSpecScore();
                    deposit.setSlideTarget(100);
                }

                if (pathTimer.getElapsedTimeSeconds() > 3.2 && pathTimer.getElapsedTimeSeconds() < 4.3) {
                    deposit.setSlideTarget(460);
                }

                if (pathTimer.getElapsedTimeSeconds() > 4.6) {
                    deposit.setSlideTarget(230);
                    if (deposit.liftPos < 250) {
                        endEffector.openClaw();
                        setPathState(pathState + 1); // Transition to next return
                    }
                }
                break;
            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(score4, true); // Follow score2 path
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.8 && pathTimer.getElapsedTimeSeconds() < 2.3) {
                    endEffector.closeClaw();
                    deposit.setSlideTarget(110);
                }

                if (pathTimer.getElapsedTimeSeconds() > 2.3 && pathTimer.getElapsedTimeSeconds() < 3) {
                    endEffector.setSpecScore();
                    deposit.setSlideTarget(100);
                }

                if (pathTimer.getElapsedTimeSeconds() > 3.2 && pathTimer.getElapsedTimeSeconds() < 4.3) {
                    deposit.setSlideTarget(460);
                }

                if (pathTimer.getElapsedTimeSeconds() > 4.6) {
                    deposit.setSlideTarget(230);
                    if (deposit.liftPos < 250) {
                        endEffector.openClaw();
                        setPathState(pathState + 1); // Transition to next return
                    }
                }
                break;
            default:
                break;
        }
    }

    private void intake(int n) {
        if (!follower.isBusy() || follower.getVelocityMagnitude() < 0.05) {
            deposit.setSlideTarget(0);
            deposit.setPivotTarget(90);
            endEffector.closeClaw();
            endEffector.setWallIntakePositionAlt();

            switch (n) {
                case 1:
                    follower.followPath(return1, true); // Follow return1 path
                    break;
                case 2:
                    follower.followPath(return2, true); // Follow return2 path
                    break;
                case 3:
                    follower.followPath(return3, true); // Follow return2 path
                    break;
                case 4:
                    follower.followPath(return4, true); // Follow return2 path
                    break;
                default:
                    break;
            }
            setPathState(pathState + 1); // Transition to next scoring path
        }
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

        endEffector.setSpecScore();
        deposit.setPivotTarget(90);
        deposit.setSlideTarget(50);

        // Build our newly incorporated multi-step path:
        buildPaths();

        if (!deposit.slideLimit.isPressed()) {
            throw new IllegalArgumentException("Zero slides before init");
        }
    }

    @Override
    public void start() {
        endEffector.setSpecScore();
        pathTimer.resetTimer();
        setPathState(0);
    }

    private double calcY(double n) {
        return startY + n * yInc;
    }

}