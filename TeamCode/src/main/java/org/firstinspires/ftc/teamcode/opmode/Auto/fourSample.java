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
import org.firstinspires.ftc.teamcode.config.util.action.SleepAction;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.List;

@Autonomous(name = "Four Sample", group = "auto", preselectTeleOp = "TeleOp")
public class fourSample extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private EndEffector endEffector;
    private Deposit deposit;
    private int pathState;

    private PathChain preload, intake1, score1, intake2, score2, intake3, score3, park;
    private final Pose startingPose =  new Pose   (7, 113, Math.toRadians(180));

    private double sample_1_x = 17;
    public double sample_1_y = 120;
    private double sample_2_x = 17;
    private double sample_2_y = 130;
    private double sample_3_x = 17;
    private double sample_3_y = 129;
    private double score_x = 14;
    private double score_y = 131;

    private double intake_slide_extention = 557;


    /**
     * Build our complex path sequence using the same calls
     * that were in the GeneratedPath constructor.
     */
    public void buildPaths() {
        // Preload path (Line 1)
        preload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(new Pose(7, 113, Math.toRadians(180))),
                        new Point(new Pose(score_x, score_y, Math.toRadians(135)))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(new Pose(score_x, score_y, Math.toRadians(135))),
                        new Point(new Pose(sample_1_x, sample_1_y, Math.toRadians(180)))))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(new Pose(sample_1_x, sample_1_y, Math.toRadians(180))),
                        new Point(new Pose(score_x, score_y, Math.toRadians(135)))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
        intake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(new Pose(score_x, score_y, Math.toRadians(135))),
                                new Point(new Pose(sample_2_x, sample_2_y, Math.toRadians(180)))))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(new Pose(sample_2_x, sample_2_y, Math.toRadians(180))),
                        new Point(new Pose(score_x, score_y, Math.toRadians(135)))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
        intake3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(new Pose(score_x, score_y, Math.toRadians(135))),
                                new Point(new Pose(sample_3_x, sample_3_y, Math.toRadians(210)))))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(210))
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(new Pose(sample_3_x, sample_3_y, Math.toRadians(210))),
                        new Point(new Pose(score_x, score_y, Math.toRadians(135)))))
                .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(135))
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(score_x, score_y, Point.CARTESIAN)),
                        new Point(new Pose(61,121,Point.CARTESIAN)),
                        new Point(new Pose(59, 94,Point.CARTESIAN))))
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    // Start following our newly built path
                    follower.followPath(preload, 1, true);
                    endEffector.setIdlePosition();
                    setPathState();

                }
                break;
            case 1:
                if (follower.getPose().getY() > score_y-11 /*&& follower.isBusy()*/) {
                    deposit.setSlideTarget(700);
                    if(deposit.slidesReached) {
                        endEffector.setBucketScorePosition();
                        setPathState();
                    }
                }
                break;
            case 2:
                if (!follower.isBusy() /*|| deposit.slidesReached*/) {
                    endEffector.openClaw();
                    endEffector.setIdlePosition();
                    deposit.setSlideTarget(0);
                    if (deposit.slidesRetracted) {
                        deposit.setPivotTarget(10);
                        follower.followPath(intake1, 0.7, true);
                        setPathState();
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    deposit.setPivotTarget(10);
                    follower.holdPoint(new Point(sample_1_x,sample_1_y), Math.toRadians(180));
                    deposit.setSlideTarget(intake_slide_extention);
                    if (deposit.liftPos > intake_slide_extention-70) {
                        // OPENING CLAW COULD BE DANGEROUS HERE
                        endEffector.openClaw();
                        endEffector.setPreSubPickupPosition();}
                    if(deposit.slidesReached){
                        endEffector.setSubPickupPosition();
                        endEffector.closeClaw();
                        endEffector.setSafeIdle();
                        new SleepAction(1);
                        deposit.setSlideTarget(0);

                    }
                    setPathState();
                }
                break;
            case 4:
                if ((!follower.isBusy() && deposit.slidesRetracted)) {
                  deposit.setPivotTarget(90);
                    follower.followPath(score1,0.8,true);
                    setPathState();
                }
                break;
            case 5:
                if (follower.getPose().getX() < sample_1_x-1 /*&& follower.isBusy()*/) {
                    deposit.setSlideTarget(700);
                    if(deposit.slidesReached) {
                        endEffector.setBucketScorePosition();
                        setPathState();
                    }
                }
                break;
            case 6:
                if (!follower.isBusy() /*|| deposit.slidesReached*/) {
                    endEffector.openClaw();
                    endEffector.setIdlePosition();
                    deposit.setSlideTarget(0);
                    if (deposit.slidesRetracted) {
                        deposit.setPivotTarget(10);
                        follower.followPath(intake2, 0.7, true);
                        setPathState();
                    }
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    deposit.setPivotTarget(10);
                    follower.holdPoint(new Point(sample_2_x,sample_2_y), Math.toRadians(180));
                    deposit.setSlideTarget(intake_slide_extention);
                    if (deposit.liftPos > intake_slide_extention-70) {
                        // OPENING CLAW COULD BE DANGEROUS HERE
                        endEffector.openClaw();
                        endEffector.setPreSubPickupPosition();}
                    if(deposit.slidesReached){
                        endEffector.setSubPickupPosition();
                        endEffector.closeClaw();
                        endEffector.setSafeIdle();
                        new SleepAction(1);
                        deposit.setSlideTarget(0);

                    }
                    setPathState();
                }
                break;
            case 8:
                if ((!follower.isBusy() && deposit.slidesRetracted)) {
                    deposit.setPivotTarget(90);
                    follower.followPath(score2,0.8,true);
                    setPathState();
                }
                break;
            case 9:
                if ((follower.isBusy() && follower.getPose().getX() < sample_1_x-2)) {
                    deposit.setSlideTarget(700);
                    if(deposit.slidesReached) {
                        endEffector.setBucketScorePosition();
                        setPathState();
                    }
                }
                break;
            case 10:
                if (!follower.isBusy() /*|| deposit.slidesReached*/) {
                    endEffector.openClaw();
                    endEffector.setIdlePosition();
                    deposit.setSlideTarget(0);
                    if (deposit.slidesRetracted) {
                        deposit.setPivotTarget(10);
                        follower.followPath(intake3, 0.7, true);
                        setPathState();
                    }
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    deposit.setPivotTarget(10);
                    follower.holdPoint(new Point(sample_3_x,sample_3_y), Math.toRadians(210));
                    deposit.setSlideTarget(intake_slide_extention);
                    if (deposit.liftPos > intake_slide_extention-70) {
                        // OPENING CLAW COULD BE DANGEROUS HERE
                        endEffector.openClaw();
                        //will need to turn wrist
                        endEffector.setPreSubPickupPosition();
                        endEffector.setWristPosition(0.23);
                    }
                    if(deposit.slidesReached){
                        endEffector.setSubPickupPosition();
                        endEffector.closeClaw();
                        endEffector.setSafeIdle();
                        new SleepAction(1);
                        deposit.setSlideTarget(0);

                    }
                    setPathState();
                }
                break;
            case 12:
                if ((!follower.isBusy() && deposit.slidesRetracted)) {
                    deposit.setPivotTarget(90);
                    follower.followPath(score3,0.8,true);
                    setPathState();
                }
                break;
            case 13:
                if (follower.getPose().getX() < sample_1_x-1 /*&& follower.isBusy()*/) {
                    deposit.setSlideTarget(700);
                    if(deposit.slidesReached) {
                        endEffector.setBucketScorePosition();
                        setPathState();
                    }
                }
                break;
            case 14:
                if (!follower.isBusy() /*|| deposit.slidesReached*/) {
                    endEffector.openClaw();
                    endEffector.setIdlePosition();
                    deposit.setSlideTarget(0);
                    if (deposit.slidesRetracted) {
                        follower.followPath(park, 1, false);
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
        //    endEffector.update();



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
        pathTimer.resetTimer();
        setPathState(0);
    }

//    private double calcY(double n) {
//        return startY + n * yInc;
//    }





}
