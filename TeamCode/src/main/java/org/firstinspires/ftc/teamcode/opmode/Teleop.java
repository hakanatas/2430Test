package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.TriggerReader;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.config.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.config.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;
import org.firstinspires.ftc.robotcontroller.internal.GoBildaPinpointDriver;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode {

    // Hardware references
    private MecanumDrive drive;
    private Deposit slides;
    private EndEffector endEffector;
    private GoBildaPinpointDriver pinpoint;

    // Gamepad snapshots
    private GamepadEx player1 = new GamepadEx(gamepad1);
    private GamepadEx player2 = new GamepadEx(gamepad2);

    TriggerReader rightTriggerReader = new TriggerReader(
            player1, GamepadKeys.Trigger.RIGHT_TRIGGER
    );

    TriggerReader leftTriggerReader = new TriggerReader(
            player1, GamepadKeys.Trigger.LEFT_TRIGGER
    );

    // State machine variables
    private int intakeState = -1;
    private int depositState = -1;
    private boolean specScoring = true;
    private int phase = 0;  // used in multi-phase intake logic
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime depositTimer = new ElapsedTime();

    // Demo field-bound example (not actively used, can remove if unneeded)
    private static final double X_MIN = -4;
    private static final double X_MAX =  4;
    private static final double Y_MIN = -4;
    private static final double Y_MAX =  4;
    private static final double SMOOTH_ZONE = 4.0;
    private static final boolean demo = true;

    @Override
    public void init() {
        // Set up bulk caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize subsystems
        drive = new MecanumDrive();
        slides = new Deposit(hardwareMap, telemetry, false);
        endEffector = new EndEffector(hardwareMap);

        endEffector.setLight(0.5);
        EndEffector.override = true;
        drive.init(hardwareMap);

        // Pinpoint driver initialization
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        // Reset timers
        intakeTimer.reset();
        depositTimer.reset();

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Handle Spec / Samp Deposit
        if (player1.getButton(GamepadKeys.Button.TOUCHPAD_FINGER_1) && player1.getButton(GamepadKeys.Button.TOUCHPAD_FINGER_2)) {
            specScoring = !specScoring;
        }

        // Handle LED override logic
        if (EndEffector.override) {
            endEffector.setLight(0);
        } else if (endEffector.pin0() && endEffector.pin1()) {
            endEffector.setLight(0.388);
        } else if (endEffector.pin0()) {
            endEffector.setLight(0.611);
        } else if (endEffector.pin1()) {
            endEffector.setLight(0.29);
        } else {
            endEffector.setLight(0.5);
        }


        // 2) Field-relative drive / IMU Controls
        double forward = -player1.getLeftY();
        double strafe  =  player1.getLeftX();
        double rotate  =  player1.getRightX();
        Pose2D currentPose = driveFieldRelative(forward, strafe, rotate);

        // IMU Reset / Calibrate
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE) || player2.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            pinpoint.resetPosAndIMU();
        }
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE) || player2.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
            pinpoint.recalibrateIMU();
        }

        // 3) Circle button => reset states
        boolean circleJustPressed = player1.wasJustPressed(GamepadKeys.Button.CIRCLE) || player2.wasJustPressed(GamepadKeys.Button.CIRCLE);
        if (circleJustPressed) {
            endEffector.setIdlePosition();
            intakeState = -1;
            depositState = -1;
            slides.pidfActive = false;
            slides.setPivotTarget(90);
            slides.setSlideTarget(0);
        }

        boolean crossJustPressed = player1.wasJustPressed(GamepadKeys.Button.CROSS) || player2.wasJustPressed(GamepadKeys.Button.CROSS);
        boolean crossJustReleased = player1.wasJustReleased(GamepadKeys.Button.CROSS) || player2.wasJustReleased(GamepadKeys.Button.CROSS);

        if (crossJustPressed) {
            intakeState = -1;
            depositState = -1;
            slides.setPivotTarget(121);
            slides.setSlideTarget(700);
        }

        if (crossJustReleased) {
            intakeState = -1;
            depositState = -1;
            slides.setPivotTarget(121);
            slides.setSlideTarget(50);
        }



        // 4) Rising edge detection for bumpers/triggers
        boolean leftBumperJustPressed = player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER);
        boolean rightBumperJustPressed = player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER);




        boolean leftTriggerJustPressed = leftTriggerReader.wasJustPressed();
        boolean rightTriggerJustPressed = rightTriggerReader.wasJustPressed();

        // Toggle override with share
        boolean shareJustPressed = player1.wasJustPressed(GamepadKeys.Button.SHARE);
        if (shareJustPressed) {
            EndEffector.override = !EndEffector.override;
        }

        // 5) Spec Intake (left bumper + / left trigger -)
        if (leftBumperJustPressed) {
            if (specScoring) {
                intakeState = (intakeState + 1) % 4;
            } else {
                intakeState = (intakeState + 1) % 6;
            }
        }
        if (leftTriggerJustPressed) {
            if (specScoring) {
                intakeState = (intakeState - 1) % 4;
            } else {
                intakeState = (intakeState - 1) % 6;
            }
        }

        // 6) Spec Deposit (right bumper + / right trigger -)
        if (rightBumperJustPressed) {
            depositState = (depositState + 1) % 6;
        }
        if (rightTriggerJustPressed) {
            depositState = 0;
        }

        // If both states are active, reset them
        if (intakeState != -1 && depositState != -1) {
            intakeState = -1;
            depositState = -1;
        }

        // 7) DPAD Controls for Claw & Pivot/Wrist
        boolean dpadLeftJustPressed = player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT);
        boolean dpadRightJustPressed = player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT);

        boolean dpadUpJustPressed = player1.wasJustPressed(GamepadKeys.Button.DPAD_UP);
        boolean dpadDownJustPressed = player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN);
        boolean dpadUpDown = player1.isDown(GamepadKeys.Button.DPAD_UP);
        boolean dpadDownDown = player1.isDown(GamepadKeys.Button.DPAD_DOWN);

        if (dpadLeftJustPressed) {
            endEffector.openClaw();
        }
        if (dpadRightJustPressed) {
            endEffector.closeClaw();
        }

        // Pivot or Wrist control depending on pivot angle
        if (slides.pivotTarget > 45) {
            if (dpadUpDown) {
                endEffector.incrementPivotPosition(0.02);
            }
            if (dpadDownDown) {
                endEffector.decrementPivotPosition(0.02);
            }
        } else {
            if (dpadUpJustPressed) {
                endEffector.incrementWristPosition(0.1375);
            }
            if (dpadDownJustPressed) {
                endEffector.decrementWristPosition(0.1375);
            }
        }



        // More manual pivot/wrist control on gamepad2 (optional)
        if (player2.isDown(GamepadKeys.Button.DPAD_UP)) {
            endEffector.incrementWristPosition(0.02);
        }
        if (player2.isDown(GamepadKeys.Button.DPAD_DOWN)) {
            endEffector.decrementWristPosition(0.02);
        }
        if (player2.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            endEffector.incrementPivotPosition(0.02);
        }
        if (player2.isDown(GamepadKeys.Button.DPAD_LEFT)) {
            endEffector.decrementPivotPosition(0.02);
        }

        // 8) Intake State Machine
        switch (intakeState) {
            case 0:
                // pivot=10, slide=0, idle pos, open claw
                slides.setPivotTarget(10);
                slides.setSlideTarget(0);
                endEffector.setIdlePosition();
                endEffector.openClaw();
                intakeTimer.reset();
                phase = 0;
                break;

            case 1:
                // Lift to ~450
                slides.setSlideTarget(450);
                if (slides.liftPos > 380) {
                    endEffector.openClaw();
                    endEffector.setPreSubPickupPosition();
                } else if (slides.slideTarget > 300) {
                    endEffector.setIdlePosition();
                    endEffector.openClaw();
                }
                intakeTimer.reset();
                phase = 0;
                break;

            case 2:
                // Multi-phase logic
                switch (phase) {
                    case 0: // subPickup
                        if (intakeTimer.milliseconds() == 0) {
                            intakeTimer.reset();
                        }
                        endEffector.setSubPickupPosition();
                        endEffector.closeClaw();

                        if (intakeTimer.milliseconds() >= 150) {
                            phase = 1;
                            intakeTimer.reset();
                        }
                        break;
                    case 1: // deposit
                        endEffector.setSafeIdle();
                        slides.setPivotTarget(10);
                        slides.setSlideTarget(0);
                        if (intakeTimer.milliseconds() >= 200) {
                            if (!endEffector.pin0() && !endEffector.pin1()) {
                                intakeState = 1;
                            }
                        }
                        break;
                }
                break;

            case 3:
                slides.setPivotTarget(90);
                if (specScoring) {
                    endEffector.setObsDepositPosition();
                } else {
                    endEffector.setSafeIdle();
                }
                intakeTimer.reset();
                phase = 0;
                break;
            case 4:
                slides.setPivotTarget(121);
                slides.setSlideTarget(700);
                intakeTimer.reset();
                phase = 0;
                break;
            case 5:
                endEffector.openClaw();
                if (intakeTimer.milliseconds() > 300) {
                    intakeState = (intakeState + 1) % 6;
                }
                phase = 0;
                break;
            default:
                intakeTimer.reset();
                phase = 0;
                break;
        }

        // 9) Deposit State Machine
        switch (depositState) {
            case 0:
                slides.setPivotTarget(90);
                slides.setSlideTarget(0);
                endEffector.setWallIntakePositionAlt();
                endEffector.openClaw();
                depositTimer.reset();
                break;

            case 1:
                // Extend or hold in some deposit logic if needed
                break;

            case 2:
                // Example: if pins are not pressed, revert
                if (!endEffector.pin0() && !endEffector.pin1()) {
                    depositState = 0;
                    depositTimer.reset();
                    break;
                }
                endEffector.closeClaw();
                if (depositTimer.milliseconds() > 300) {
                    depositState = 2;
                    depositTimer.reset();
                }
                break;

            case 3:
                endEffector.setSpecScore();
                depositTimer.reset();
                break;

            case 4:
                slides.setSlideTarget(475);
                depositTimer.reset();
                break;

            case 5:
                slides.setSlideTarget(210);
                if (slides.liftPos < 220) {
                    endEffector.openClaw();
                }
                if (depositTimer.milliseconds() > 250) {
                    depositState = 0;
                    depositTimer.reset();
                }
                break;

            default:
                depositTimer.reset();
                break;
        }

        // Update deposit mechanics
        slides.update();

        // 10) Telemetry
        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency());
        telemetry.addData("Lift Pos", slides.liftPos);
        telemetry.addData("Lift Target", slides.slidePIDF.getSetPoint());
        telemetry.addData("Pivot Pos", slides.pivotPos);
        telemetry.addData("Pivot Target", slides.pivotTarget);
        telemetry.update();
    }

    /**
     * Configures the GoBilda Pinpoint device.
     */
    private void configurePinpoint() {
        pinpoint.setOffsets(6.03262717 * 25.4, 2.71126772 * 25.4);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
    }

    /**
     * Applies field-relative drive using the Pinpoint’s IMU heading.
     */
    private Pose2D driveFieldRelative(double forward, double right, double rotate) {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();  // Current position

        double robotAngle = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));

        // Convert driver input into field-centric motion
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // Convert to robot-centric forward & strafe
        double newForward = r * Math.sin(theta);
        double newRight   = r * Math.cos(theta);

        // Drive with final inputs
        drive.drive(newForward, newRight, rotate);
        return pos;
    }
}
