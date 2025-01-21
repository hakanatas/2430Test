package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
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
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    // State machine variables
    private int intakeState = -1;
    private int depositState = -1;
    private int phase = 0; // used in your multi-phase intake logic
    private ElapsedTime specIntakeTimer = new ElapsedTime();
    private ElapsedTime specDepoTimer = new ElapsedTime();

    private ElapsedTime liftResetTimer = new ElapsedTime();
    private boolean isLiftResetting = false;

    // Example bounding box, in inches, around the origin or wherever you’ve chosen:
    private static final double X_MIN = -24;  // left boundary
    private static final double X_MAX =  24;  // right boundary
    private static final double Y_MIN = -24;  // bottom boundary
    private static final double Y_MAX =  24;  // top boundary
    private static final double SMOOTH_ZONE = 4.0;
    private static final boolean demo = false;


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


        drive.init(hardwareMap);
        endEffector.setIdlePosition(); // set the end effector to idle position

        // Pinpoint driver initialization
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        // Reset your timers, etc.
        specIntakeTimer.reset();
        specDepoTimer.reset();

        telemetry.addLine("Ready!");
        telemetry.update();
    }




    @Override
    public void loop() {

        if (EndEffector.override) {
            endEffector.setLight(0);
        }
        else if (endEffector.pin0() && endEffector.pin1()) {
            endEffector.setLight(0.388);
        } else if (endEffector.pin0()) {
            endEffector.setLight(0.611);
        } else if (endEffector.pin1()) {
            endEffector.setLight(0.29);
        } else {
            endEffector.setLight(0.5);
        }

        //------------------------------------------
        // 1) Copy previous -> current gamepad
        //------------------------------------------
        // First, remember what the previous state was
        previousGamepad1.copy(currentGamepad1);
        // Then copy the new hardware state into current
        currentGamepad1.copy(gamepad1);

        //------------------------------------------
        // 2) Drive / IMU Controls
        //------------------------------------------
        double forward = -currentGamepad1.left_stick_y;
        double strafe  =  currentGamepad1.left_stick_x;
        double rotate  =  currentGamepad1.right_stick_x;

        Pose2D currentPose = driveFieldRelative(forward, strafe, rotate);

        // If x is pressed, reset IMU
        if (currentGamepad1.square) {
            pinpoint.resetPosAndIMU();
        }
        // If y is pressed, recalibrate IMU
        if (currentGamepad1.triangle) {
            pinpoint.recalibrateIMU();
        }
        // Circle button logic
        boolean circleJustPressed = currentGamepad1.circle && !previousGamepad1.circle;
        boolean circleReleased = !currentGamepad1.circle && previousGamepad1.circle;
        if (circleJustPressed) {
            endEffector.setIdlePosition();
            intakeState = -1;
            depositState = -1;
            slides.pidfActive = false;
            slides.setPivotTarget(90);
            slides.setSlideTarget(0);
        }


        //------------------------------------------
        // 3) Rising Edge Detection for Bumpers/Triggers
        //------------------------------------------
        boolean leftBumperJustPressed =
                (currentGamepad1.left_bumper && !previousGamepad1.left_bumper);
        boolean rightBumperJustPressed =
                (currentGamepad1.right_bumper && !previousGamepad1.right_bumper);

        boolean leftTriggerJustPressed =
                (currentGamepad1.left_trigger > 0.5f) && (previousGamepad1.left_trigger <= 0.5f);
        boolean rightTriggerJustPressed =
                (currentGamepad1.right_trigger > 0.5f) && (previousGamepad1.right_trigger <= 0.5f);

        boolean shareJustPressed = (currentGamepad1.share && !previousGamepad1.share);
        if (shareJustPressed) {
            EndEffector.override = !EndEffector.override;
        }


//        if (circleJustPressed) {
//            // Stop PIDF control
//            slides.stopPIDF();
//            // Set pivot target and idle position
//            slides.setPivotTarget(90);
//            endEffector.setIdlePosition();
//            // Move the lift downward at -0.5 power
//            slides.setLiftPower(-0.5);
//            isLiftResetting = false; // Reset flag
//        }
//
//        if (currentGamepad1.circle) {
//            // Continue holding lift power while the circle is pressed
//            slides.setLiftPower(-0.5);
//        }
//
//        if (circleReleased) {
//            // Stop lift motor and reset encoder
//            slides.setLiftPower(0);
//            slides.resetLiftEncoder();
//
//            // Start the timer for the delay
//            liftResetTimer.reset();
//            isLiftResetting = true;
//        }
//
//        // Check if the delay has elapsed
//        if (isLiftResetting && liftResetTimer.milliseconds() >= 150) {
//            slides.startPIDF();
//            isLiftResetting = false; // Reset the flag
//        }



        //------------------------------------------
        // 4) Spec Intake (left bumper +/ left trigger -)
        //------------------------------------------
        if (leftBumperJustPressed) {
            intakeState = (intakeState + 1) % 4; // cycle 0..3
        }
        if (leftTriggerJustPressed) {
            intakeState = (intakeState - 1) % 4; // cycle 0..3
        }

        //------------------------------------------
        // 5) Spec Deposit (right bumper +/ right trigger -)
        //------------------------------------------
        if (rightBumperJustPressed) {
            depositState = (depositState + 1) % 5; // cycle 0..4
        }
        if (rightTriggerJustPressed) {
            depositState = 0; // cycle 0..4
        }

        // If both states are active, reset them (example logic — adjust to your liking)
        if (intakeState != -1 && depositState != -1) {
            intakeState = -1;
            depositState = -1;
        }

        //------------------------------------------
        // 6) DPAD Controls for the Claw & Pivot/Wrist
        //------------------------------------------
        boolean dpadLeftJustPressed =
                (currentGamepad1.dpad_left && !previousGamepad1.dpad_left);
        boolean dpadRightJustPressed =
                (currentGamepad1.dpad_right && !previousGamepad1.dpad_right);
        boolean dpadUpJustPressed =
                (currentGamepad1.dpad_up && !previousGamepad1.dpad_up);
        boolean dpadDownJustPressed =
                (currentGamepad1.dpad_down && !previousGamepad1.dpad_down);

        // --- Claw Open/Close ---
        if (dpadLeftJustPressed) {
            endEffector.openClaw();
        }
        if (dpadRightJustPressed) {
            endEffector.closeClaw();
        }

        // --- Pivot or Wrist Control ---
        if (slides.pivotTarget > 45) {
            // Control pivot
            if (currentGamepad1.dpad_up) {
                endEffector.incrementPivotPosition(0.02);
            }
            if (currentGamepad1.dpad_down) {
                endEffector.decrementPivotPosition(0.02);
            }
        } else {
            // Control wrist
            if (currentGamepad1.dpad_up) {
                endEffector.incrementWristPosition(0.02);
            }
            if (currentGamepad1.dpad_down) {
                endEffector.decrementWristPosition(0.02);
            }
        }

        //------------------------------------------
        // 7) Intake State Machine
        //------------------------------------------
        switch (intakeState) {
            case 0:
                // pivot=0, slide=50, idle pos, open claw
                slides.setPivotTarget(10);
                slides.setSlideTarget(50);
                endEffector.setIdlePosition();
                endEffector.openClaw();
                specIntakeTimer.reset();
                phase = 0;
                break;

            case 1:
                // move lift to 400
                slides.setSlideTarget(450);
                if (slides.liftPos > 380) {
                    // OPENING CLAW COULD BE DANGEROUS HERE
                    endEffector.openClaw();
                    endEffector.setPreSubPickupPosition();
                } else if (slides.slideTarget > 300) {
                    endEffector.setIdlePosition();
                    endEffector.openClaw();
                }

                specIntakeTimer.reset();
                phase = 0;
                break;

            case 2:
                // multi-phase logic
                switch (phase) {
                    case 0: // subPickup
                        if (specIntakeTimer.milliseconds() == 0) {
                            specIntakeTimer.reset();
                        }
                        endEffector.setSubPickupPosition();
                        endEffector.closeClaw();

                        if (specIntakeTimer.milliseconds() >= 150) {
                            phase = 1;
                            specIntakeTimer.reset();
                        }
                        break;
                    case 1: // deposit
                        endEffector.setSafeIdle();
                        slides.setPivotTarget(10);
                        slides.setSlideTarget(50);
                        if (specIntakeTimer.milliseconds() >= 150) {
                            if (!endEffector.pin0() && !endEffector.pin1()) {
                                intakeState = 1;
                            }
                        }
                        break;
                }
                break;

            case 3:
                slides.setPivotTarget(90);
                endEffector.setObsDepositPosition();
                specIntakeTimer.reset();
                phase = 0;
                break;

            default:
                specIntakeTimer.reset();
                phase = 0;
                break;
        }

        //------------------------------------------
        // 8) Deposit State Machine
        //------------------------------------------
        switch (depositState) {
            case 0:
                slides.setPivotTarget(90);
                slides.setSlideTarget(0);
                endEffector.setWallIntakePositionAlt();
                endEffector.openClaw();
                specDepoTimer.reset();
                break;

            case 1:
                if (!endEffector.pin0() && !endEffector.pin1()) {
                    depositState = 0;
                    specDepoTimer.reset();
                    break;
                }
                endEffector.closeClaw();
                if (specDepoTimer.milliseconds() > 300) {
                    depositState = 2;
                    specDepoTimer.reset();
                }
                break;
            case 2:
                endEffector.setSpecScore();
                specDepoTimer.reset();
                break;

            case 3:
                slides.setSlideTarget(475);
                specDepoTimer.reset();
                break;

            case 4:
                slides.setSlideTarget(210);
                if (slides.liftPos < 230) {
                    endEffector.openClaw();
                }
                if (specDepoTimer.milliseconds() > 250) {
                    depositState = 0;
                    specDepoTimer.reset();
                }
                break;
            default:
                specDepoTimer.reset();
                break;
        }

        // Update deposit mechanics
        slides.update();

        //------------------------------------------
        // 9) Telemetry
        //------------------------------------------
        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency());
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Deposit State", depositState);
        telemetry.addData("At set point", slides.slidesReached);
        telemetry.addData("Lift Pos", slides.liftPos);
        telemetry.addData("Lift Target", slides.slidePIDF.getSetPoint());
        telemetry.addData("Pivot Pos", slides.pivotPos);
        telemetry.addData("Pivot Target", slides.pivotTarget);
        telemetry.addData("Error", slides.slidePIDF.getPositionError());
        telemetry.addData("Pin0", endEffector.pin0());
        telemetry.addData("Pin1", endEffector.pin1());
        telemetry.update();
    }

    // ------------------------------------------------
    // HELPER: Configure the GoBilda Pinpoint device
    // ------------------------------------------------
    private void configurePinpoint() {
        pinpoint.setOffsets(153.22873, -68.86620);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
    }

    // ------------------------------------------------
    // HELPER: Field-relative drive
    // ------------------------------------------------
//    private Pose2D driveFieldRelative(double forward, double right, double rotate) {
//        pinpoint.update();
//        Pose2D pos = pinpoint.getPosition();  // Current position
//
//        double robotAngle = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));
//        double theta = Math.atan2(forward, right);
//        double r = Math.hypot(forward, right);
//        theta = org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
//                .normalizeRadians(theta - robotAngle);
//
//        double newForward = r * Math.sin(theta);
//        double newRight   = r * Math.cos(theta);
//
//        if (demo) {
//
//        }
//
//        drive.drive(newForward, newRight, rotate);
//        return pos;
//    }



    private Pose2D driveFieldRelative(double forward, double right, double rotate) {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();  // Current position

        double robotAngle = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));

        // Convert driver input into field-centric motion
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // These are the "robot-centric" forward & strafe after field correction
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        double xPos = pos.getX(DistanceUnit.INCH);
        double yPos = pos.getY(DistanceUnit.INCH);


        if (demo) {
            // ------------------------------
            // X-AXIS SMOOTH CLAMP
            // ------------------------------

            // Left boundary
            if (xPos < X_MIN) {
                // We are already past the boundary. Stop any further left motion.
                if (newRight < 0) newRight = 0;
            } else {
                // We are inside or near the boundary
                double distFromLeft = xPos - X_MIN;  // how far from X_MIN
                if (distFromLeft < SMOOTH_ZONE) {
                    // If user is trying to move further left (negative right), scale the input
                    if (newRight < 0) {
                        double scale = distFromLeft / SMOOTH_ZONE;    // between 0..1
                        newRight *= clamp(scale, 0, 1);
                    }
                }
            }

            // Right boundary
            if (xPos > X_MAX) {
                // Already past the boundary
                if (newRight > 0) newRight = 0;
            } else {
                // Within or near the boundary
                double distFromRight = X_MAX - xPos;  // how far from X_MAX
                if (distFromRight < SMOOTH_ZONE) {
                    // If user is trying to move further right (positive right), scale
                    if (newRight > 0) {
                        double scale = distFromRight / SMOOTH_ZONE;   // 0..1
                        newRight *= clamp(scale, 0, 1);
                    }
                }
            }

            // ------------------------------
            // Y-AXIS SMOOTH CLAMP
            // ------------------------------

            // Bottom boundary
            if (yPos < Y_MIN) {
                if (newForward < 0) newForward = 0;
            } else {
                double distFromBottom = yPos - Y_MIN;
                if (distFromBottom < SMOOTH_ZONE) {
                    if (newForward < 0) {
                        double scale = distFromBottom / SMOOTH_ZONE;
                        newForward *= clamp(scale, 0, 1);
                    }
                }
            }

            // Top boundary
            if (yPos > Y_MAX) {
                if (newForward > 0) newForward = 0;
            } else {
                double distFromTop = Y_MAX - yPos;
                if (distFromTop < SMOOTH_ZONE) {
                    if (newForward > 0) {
                        double scale = distFromTop / SMOOTH_ZONE;
                        newForward *= clamp(scale, 0, 1);
                    }
                }
            }
        }
        // Now drive with our final scaled inputs
        drive.drive(newForward, newRight, rotate);

        return pos;
    }

    // Simple clamp helper
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}