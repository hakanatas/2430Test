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

@Autonomous(name = "fiveSpec", group = "auto")
public class fiveSpec extends OpMode {
    private ElapsedTime timer = new ElapsedTime();
    private Follower follower;
    private Timer pathTimer;
    private EndEffector endEffector;
    private Deposit deposit;
    private int pathState;

    private PathChain preload, pushSample1Path, pushSample2Path, pushSample3Path, combinedPush, score1, return1, score2, return2, score3, return3, score4, return4, shift;
   private Pose startingPose =  new Pose   (7, 65, Math.toRadians(0));


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
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        combinedPush = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(40.000, 65.859, Math.toRadians(0))),
                        new Point(new Pose(34.000, 67.000, Math.toRadians(0))),
                        new Point(new Pose(0.000, 48.000, Math.toRadians(0))),
                        new Point(new Pose(55, 30.000, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(55, 30.000, Math.toRadians(0))),
                        new Point(new Pose(65.000, 20.000, Math.toRadians(0))),
                        new Point(new Pose(22, 24.000, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(22, 24.000, Math.toRadians(0))),
                        new Point(new Pose(65.000, 28, Math.toRadians(0))),
                        new Point(new Pose(55, 13.5, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(new Pose(55, 13.5, Math.toRadians(0))),
                        new Point(new Pose(22, 13.5, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(22, 12.500, Math.toRadians(0))),
                        new Point(new Pose(65.000, 16.250, Math.toRadians(0))),
                        new Point(new Pose(50, 8.75, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(new Pose(50, 8.75, Math.toRadians(0))),
                        new Point(new Pose(8.5, 8.75, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        // Push Sample 1 Path (Lines 2 and 3)
        pushSample1Path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(40.000, 65.859, Math.toRadians(0))),
                        new Point(new Pose(34.000, 67.000, Math.toRadians(0))),
                        new Point(new Pose(0.000, 48.000, Math.toRadians(0))),
                        new Point(new Pose(55, 30.000, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(55, 30.000, Math.toRadians(0))),
                        new Point(new Pose(65.000, 20.000, Math.toRadians(0))),
                        new Point(new Pose(13.250, 24.000, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Push Sample 2 Path (Lines 4 and 5)
        pushSample2Path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(13.250, 24.000, Math.toRadians(0))),
                        new Point(new Pose(65.000, 28, Math.toRadians(0))),
                        new Point(new Pose(55, 13.5, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(new Pose(55, 13.5, Math.toRadians(0))),
                        new Point(new Pose(13.250, 13.5, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Push Sample 3 Path (Lines 6 and 7)
        pushSample3Path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(13.250, 12.500, Math.toRadians(0))),
                        new Point(new Pose(65.000, 16.250, Math.toRadians(0))),
                        new Point(new Pose(50, 8.75, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(new Pose(50, 8.75, Math.toRadians(0))),
                        new Point(new Pose(8.5, 8.75, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(8.5, 8, Math.toRadians(0))),
                        new Point(new Pose(36, 24, Math.toRadians(0))),
                        new Point(new Pose(12, 48, Math.toRadians(0))),
                        new Point(new Pose(40, 68, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        return1 = follower.pathBuilder()
                .addPath(
                    new BezierCurve(
                        new Point(40.000, 68.000, Point.CARTESIAN),
                        new Point(4.500, 72.000, Point.CARTESIAN),
                        new Point(42.000, 24.000, Point.CARTESIAN),
                        new Point(8.000, 28.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        score2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(8.000, 28.000, Point.CARTESIAN),
                                new Point(42.000, 24.000, Point.CARTESIAN),
                                new Point(4.500, 72.000, Point.CARTESIAN),
                                new Point(38.000, 68.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(38, 70, Point.CARTESIAN), new Point(40, 68.5, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        return2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(38.000, 68.000, Point.CARTESIAN),
                                new Point(4.500, 72.000, Point.CARTESIAN),
                                new Point(42.000, 24.000, Point.CARTESIAN),
                                new Point(8.000, 28.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    // Start following our newly built path
                    follower.followPath(preload, true);

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
                    follower.followPath(combinedPush,true);
                    setPathState(4);
            }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(pushSample2Path, false);
                    setPathState(pathState + 1);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(pushSample3Path, true);
                    setPathState(pathState + 1);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    endEffector.closeClaw();
                    deposit.setSlideTarget(50);
                    setPathState(pathState + 1);
                }
                break;
            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(score1, true); // Follow score1 path
                }

                if (pathTimer.getElapsedTimeSeconds() > 0.3 && pathTimer.getElapsedTimeSeconds() < 1) {
                    endEffector.setSpecScore(); // Adjust claw for scoring
                    deposit.setSlideTarget(100); // Set initial slide position
                }

                if (pathTimer.getElapsedTimeSeconds() > 1.2 && pathTimer.getElapsedTimeSeconds() < 2.5) {
                    deposit.setSlideTarget(475); // Lift slide to scoring position
                }

                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    deposit.setSlideTarget(230); // Lower slide slightly for object release
                    if (deposit.liftPos < 250) {
                        endEffector.openClaw(); // Release object
                        setPathState(pathState + 1); // Transition to return state
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    deposit.setSlideTarget(0); // Reset slide to initial position
                    deposit.setPivotTarget(90); // Reset pivot
                    endEffector.setWallIntakePositionAlt(); // Prepare claw for intake
                    follower.followPath(return1, true); // Follow return1 path
                    setPathState(pathState + 1); // Transition to the next scoring path
                }
                break;
            case 7:
                cycle();
                break;
            case 8:
                intake();
                break;
            case 9:
                cycle();
                break;

            case 10:
                intake();
                break;
            case 11:
                cycle();
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

    private void cycle() {

        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
            follower.followPath(score2, true); // Follow score2 path
        }
        if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 2.2) {
            deposit.setSlideTarget(75);
            endEffector.closeClaw();
        }

        if (pathTimer.getElapsedTimeSeconds() > 2.3 && pathTimer.getElapsedTimeSeconds() < 3) {
            endEffector.setSpecScore();
            deposit.setSlideTarget(100);
        }

        if (pathTimer.getElapsedTimeSeconds() > 3.2 && pathTimer.getElapsedTimeSeconds() < 4.5) {
            deposit.setSlideTarget(475);
        }

        if (pathTimer.getElapsedTimeSeconds() > 4.75) {
            deposit.setSlideTarget(230);
            if (deposit.liftPos < 250) {
                endEffector.openClaw();
                setPathState(pathState + 1); // Transition to next return
            }
        }
    }

    private void intake() {
        if (!follower.isBusy()) {
            deposit.setSlideTarget(0);
            deposit.setPivotTarget(90);
            endEffector.closeClaw();
            endEffector.setWallIntakePositionAlt();
            follower.followPath(return2, true); // Follow return2 path
            setPathState(pathState + 1); // Transition to next scoring path
        }
    }

    private void preCycle() {
        if (!follower.isBusy()) {
            endEffector.closeClaw();
            deposit.setSlideTarget(50);
            setPathState(pathState + 1);
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


        telemetry.update();

        if (follower.getPose().getX()<=36 && follower.getPose().getY() <= 36) {
            follower.setMaxPower(0.6);
        } else {
            follower.setMaxPower(0.9);
        }

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

        deposit = new Deposit(hardwareMap, telemetry, true);
        endEffector = new EndEffector(hardwareMap);

        endEffector.setSpecScore();
        deposit.setPivotTarget(90);
        deposit.setSlideTarget(50);

        // Build our newly incorporated multi-step path:
        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }
}