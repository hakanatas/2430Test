package org.firstinspires.ftc.teamcode.opmode.Auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.Point;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.config.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.List;


public class Poses {
    // Preload Pathchain

    public static final double farX = 50;
    public static final double closeX = 24;
    public static final Pose startPose = new Pose(7.3285, 78.1415, 0);
    public static final Pose preloadPose = new Pose(40, 78.1415, 0);

    // Combined Push Pathchain
    public static final Pose spline1Control = new Pose(24, 32, 0);
    public static final Pose spline1End = new Pose(farX, 32, 0);

    public static final Pose spline2Control = new Pose(farX, 22.5, 0);
    public static final Pose spline2End = new Pose(closeX, 22.5, 0);

    public static final Pose spline3Control = new Pose(farX, 22.5, 0);
    public static final Pose spline3End = new Pose(farX, 12.5, 0);

    public static final Pose pushSample2End = new Pose(closeX, 12.5, 0);

    public static final Pose spline4Control = new Pose(farX, 16.5, 0);
    public static final Pose spline4End = new Pose(farX, 7, 0);

    public static final Pose pushSample3End = new Pose(closeX, 7, 0);

    public static final Pose spline5Control = new Pose(closeX, 36, 0);
    public static final Pose spline5End = new Pose(7.3285, 36, 0);

    public static final Pose scoreEnd = new Pose (40, 70, 0);

}
