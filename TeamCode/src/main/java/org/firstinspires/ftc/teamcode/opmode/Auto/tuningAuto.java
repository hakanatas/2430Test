package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystems.PivotExtension;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

@Autonomous(name = "tuningAuto")
public class tuningAuto extends OpMode{
    private ElapsedTime timer = new ElapsedTime();
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private PivotExtension slides;

    private Pose startPos = new Pose(0,0, Math.toRadians(0));
    private Pose interPos = new Pose(2, -2, Math.toRadians(90));
    private Pose endPos = new Pose(2, 2, Math.toRadians(45));

    private PathChain path;

    public void buildPaths() {
        path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(interPos)))
                .setLinearHeadingInterpolation(startPos.getHeading(), interPos.getHeading())
                .addPath(new BezierLine(new Point(interPos), new Point(endPos)))
                .setLinearHeadingInterpolation(interPos.getHeading(), endPos.getHeading())
                .addPath(new BezierLine(new Point(endPos), new Point(startPos)))
                .setLinearHeadingInterpolation(endPos.getHeading(), startPos.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    follower.followPath(path, true);

                    setPathState(-1);
                }
                break;
            default:
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {
        follower.update();
        slides.update();

        autonomousPathUpdate();
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        follower = new Follower(hardwareMap);
        slides = new PivotExtension(hardwareMap, telemetry, true);
        follower.setStartingPose(startPos);
        buildPaths();

    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

}
