package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.lynx.LynxModule;

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

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Modular Five Spec - Revised", group = "auto", preselectTeleOp = "Teleop")
public class ModularFiveSpec extends OpMode {
    // Constants
    private static final Pose STARTING_POSE = new Pose(7, 65.859, Math.toRadians(0));
    private static final double WALL_INTAKE_X = 7;
    private static final double WALL_OFFSET = 1.2;
    private static final double SUBSTATION_X = 39.7;
    private static final double START_Y = 65.859;
    private static final double Y_INCREMENT = 2;
    private static final double SPLINE_CONTROL = 22;
    // Use slide positions as needed
    private static final int INTAKE_SLIDE_POSITION = 215;

    // Subsystems
    private Follower follower;
    private Timer pathTimer;
    private EndEffector endEffector;
    private Deposit deposit;

    // New state management
    private enum AutoState {
        PRELOAD,            // Follow preload path
        PUSH,               // Follow push path
        ALIGNING,           // Alignment routine for intake (both after push and in cycles)
        PIVOT_ADJUSTING,    // Optional pivot adjustment if alignment sensor is not satisfied
        SCORE_CYCLE,        // Follow score path during cycle and then score the piece
        RETURN_CYCLE,       // Follow return path in cycle
        FINISHED            // Finished autonomous
    }

    private AutoState currentState = AutoState.PRELOAD;
    // Cycle count for cycles after push (we want 3 cycles)
    private int cycleCount = 0;
    private static final int MAX_CYCLES = 4;

    // Path sequence list
    private List<PathChain> pathSequence = new ArrayList<>();
    private int currentPathIndex = 0;

    // Alignment parameters (used in both push and cycle intakes)
    private boolean alignmentChecked = false;
    private static final int ALIGNMENT_PIVOT_ANGLE = 121;
    private static final int ALIGNMENT_WAIT_MS = 200;

    @Override
    public void init() {
        setupHardware();
        buildPathSequence();
        initializeSubsystems();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        startNextPath();
    }

    @Override
    public void loop() {
        follower.update();
        updateSubsystems();
        handleStateMachine();
        updateTelemetry();
    }

    private void setupHardware() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(STARTING_POSE);
        follower.setMaxPower(1);

        deposit = new Deposit(hardwareMap, telemetry, true);
        endEffector = new EndEffector(hardwareMap);
        pathTimer = new Timer();
    }

    private void initializeSubsystems() {
        endEffector.setLight(0.5);
        EndEffector.override = false;
        endEffector.init();
        deposit.setPivotTarget(90);
        deposit.setSlideTarget(50);
    }

    /**
     * Build the path sequence in the order you want:
     * 0. Preload path
     * 1. Push path
     * Then for each cycle (3 cycles):
     * 2. Score path (cycle 1), 5. Score path (cycle 2), 8. Score path (cycle 3)
     * 3. Return path (cycle 1), 6. Return path (cycle 2), 9. Return path (cycle 3)
     * 4. Intake path (cycle 1), 7. Intake path (cycle 2), 10. Intake path (cycle 3)
     */
    private void buildPathSequence() {
        // Preload path (index 0)
        pathSequence.add(follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(STARTING_POSE),
                        new Point(40, START_Y, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build());

        // Push path (index 1)
        pathSequence.add(follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(40.000, START_Y, Math.toRadians(0))),
                        new Point(new Pose(34.000, 67.000, Math.toRadians(0))),
                        new Point(new Pose(0.000, 48.000, Math.toRadians(0))),
                        new Point(new Pose(50, 30.000, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                // You can add additional segments if needed
                .build());

        // For each cycle (3 cycles) add a score path, return path, and intake path.
        for (int i = 1; i <= MAX_CYCLES; i++) {
            double targetY = START_Y + i * Y_INCREMENT;
            // Score path
            pathSequence.add(follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Point(WALL_INTAKE_X, 30, Point.CARTESIAN),
                            new Point(SUBSTATION_X, 30, Point.CARTESIAN),
                            new Point(SPLINE_CONTROL, targetY, Point.CARTESIAN),
                            new Point(SUBSTATION_X, targetY, Point.CARTESIAN)))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build());

            // Return path
            pathSequence.add(follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Point(SUBSTATION_X, targetY, Point.CARTESIAN),
                            new Point(SPLINE_CONTROL, targetY, Point.CARTESIAN),
                            new Point(SUBSTATION_X, 30, Point.CARTESIAN),
                            new Point(WALL_INTAKE_X + 10, 30, Point.CARTESIAN)))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build());

            // Intake path
            pathSequence.add(follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Point(WALL_INTAKE_X + WALL_OFFSET, 30, Point.CARTESIAN),
                            new Point(WALL_INTAKE_X + 10, 30, Point.CARTESIAN)))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build());
        }
    }

    // --- State Machine ---
    private void handleStateMachine() {
        switch (currentState) {
            case PRELOAD:
                if (follower.getPose().getX() > 10 && follower.isBusy()) {
                    deposit.setSlideTarget(490);
                }
                if (!follower.isBusy()) {
                    handlePreloadCompletion();
                }
                break;
            case PUSH:
                if (!follower.isBusy()) {
                    // After push path, start the intake/alignment routine for pickup.
                    prepareForIntake();
                    currentState = AutoState.ALIGNING;
                }
                break;
            case ALIGNING:
                handleAligning();
                break;
            case PIVOT_ADJUSTING:
                handlePivotAdjusting();
                break;
            case SCORE_CYCLE:
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
                        handleScoreCycleCompletion();
                    }
                }
                break;
            case RETURN_CYCLE:
                if (!follower.isBusy()) {
                    handleReturnCycleCompletion();
                }
                break;
            default:
                break;
        }
    }

    // --- Handler Methods ---

    // Called when preload path is finished.
    private void handlePreloadCompletion() {
        // Score the preloaded piece
        if (follower.getPose().getX() >= 19 && follower.getVelocityMagnitude() < 0.5) {
            deposit.setSlideTarget(230);
            if (deposit.liftPos < 250) {
                endEffector.openClaw();
                currentState = AutoState.PUSH;
                startNextPath();
            }
        }

    }

    // Called when push path finishes.
    // prepareForIntake() is already called in PUSH, so we use the ALIGNING routine.

    // Alignment routine used for both the push intake and cycle intakes.
    private void handleAligning() {
        if (!alignmentChecked) {
            // Check if sensor condition is met (using endEffector.either())
            if (endEffector.either()) {
                // Alignment complete: pick up the piece
                endEffector.closeClaw();
                deposit.setSlideTarget(INTAKE_SLIDE_POSITION);
                // If we are coming from the push path, move into the cycle phase.
                // Otherwise, we are in a cycle intake.
                currentState = AutoState.SCORE_CYCLE;
                // Increment cycle count if this was a cycle intake (not the push intake)
                if (cycleCount > 0 || cycleCount == 0) {
                    cycleCount++;  // For push intake, this makes cycleCount==1.
                }
                alignmentChecked = false;
                startNextPath();
            } else {
                // If not aligned, try a pivot adjustment
                deposit.setPivotTarget(ALIGNMENT_PIVOT_ANGLE);
                pathTimer.resetTimer();
                alignmentChecked = true;
                currentState = AutoState.PIVOT_ADJUSTING;
            }
        }
    }

    private void handlePivotAdjusting() {
        if (pathTimer.getElapsedTime() >= ALIGNMENT_WAIT_MS) {
            // After waiting, reset the pivot
            deposit.setPivotTarget(90);
            alignmentChecked = false;
            currentState = AutoState.ALIGNING;
        }
    }

    // Called when a cycle's score path is finished.
    private void handleScoreCycleCompletion() {
        // Score the piece
        endEffector.openClaw();
        deposit.setSlideTarget(0);
        // Next, transition to the return phase of this cycle.
        currentState = AutoState.RETURN_CYCLE;
        startNextPath();
    }

    // Called when a cycle's return path is finished.
    private void handleReturnCycleCompletion() {
        // After returning, prepare for intake.
        prepareForIntake();
        // If we have completed all cycles, we finish.
        if (cycleCount >= MAX_CYCLES + 1) {  // cycleCount was incremented after the push intake too.
            currentState = AutoState.FINISHED;
        } else {
            currentState = AutoState.ALIGNING;
            startNextPath();
        }
    }

    // Prepares subsystems for an intake routine.
    private void prepareForIntake() {
        if (pathTimer.getElapsedTimeSeconds() > 1 && pathTimer.getElapsedTimeSeconds() < 2) {
            deposit.setPivotTarget(90);
            deposit.setPivotTarget(0);
            endEffector.setWallIntakePositionAlt();
            alignmentChecked = false;  // Reset for the new intake
        }

    }

    // --- Path Handling ---
    private void startNextPath() {
        if (currentPathIndex < pathSequence.size()) {
            // Only follow a new path if not in an alignment waiting state.
            if (currentState != AutoState.ALIGNING && currentState != AutoState.PIVOT_ADJUSTING) {
                follower.followPath(pathSequence.get(currentPathIndex), 1, false);
                currentPathIndex++;
            }
            pathTimer.resetTimer();
        }
    }

    // --- Subsystem and Telemetry Updates ---
    private void updateSubsystems() {
        deposit.update();
    }

    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Cycle", cycleCount);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Slide Pos", deposit.liftPos);
        telemetry.update();
    }

    @Override
    public void stop() {
        endEffector.openClaw();
    }
}
