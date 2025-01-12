package org.firstinspires.ftc.teamcode.config.subsystems.Folder;

import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class pivotSlidesArchive {
    // Hardware
    private final DcMotorEx pivotMotor;
    private final DcMotorEx extensionMotorLeft;
    private final DcMotorEx extensionMotorRight;
    private final AnalogInput pivotEncoder;
    private final Gamepad gamepad;

    // PIDF Controllers
    private final PIDController pivotPID;
    private final PIDController extensionPID;

    // PID Coefficients
    public static double PIVOT_KP = 0.02, PIVOT_KI = 0.0, PIVOT_KD = 0.001, PIVOT_KF = 0.0;
    public static double EXTENSION_KP = 0.01, EXTENSION_KI = 0.0, EXTENSION_KD = 0.001, EXTENSION_KF = 0.0, EXTENSION_CONSTANT=0;


    // State Variables
    private double pivotTargetAngle = 0.0;
    private double extensionTargetLength = 0.0;
    private boolean zeroing = false;


    public pivotSlidesArchive(HardwareMap hardwareMap, Gamepad gamepad) {
        pivotMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extensionMotorLeft = hardwareMap.get(DcMotorEx.class, "extensionLeft");
        extensionMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extensionMotorRight = hardwareMap.get(DcMotorEx.class, "extensionRight");
        extensionMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivot_enc");

        this.gamepad = gamepad;

        pivotPID = new PIDController(PIVOT_KP, PIVOT_KI, PIVOT_KD);
        extensionPID = new PIDController(EXTENSION_KP, EXTENSION_KI, EXTENSION_KD);

        pivotTargetAngle = getPivotAngle();
        extensionTargetLength = getExtensionLength();
    }

    public void update() {
        // Feedforward for pivot
        PIVOT_KF = Math.cos(Math.toRadians(getPivotAngle())) * linearScaledFeedForward();

        // Feedforward for extension
        EXTENSION_KF = EXTENSION_CONSTANT * Math.sin(Math.toRadians(getPivotAngle()));

        // Calculate motor outputs
        double pivotOutput = pivotPID.calculate(getPivotAngle(), pivotTargetAngle) + PIVOT_KF;
        double extensionOutput = extensionPID.calculate(getExtensionLength(), extensionTargetLength) + EXTENSION_KF;

        // Set motor outputs
        if (Math.abs(gamepad.left_stick_y) > 0.5) {
            pivotOutput = gamepad.left_stick_y > 0.5 ? 0.1 : -0.1;
            pivotMotor.setPower(pivotOutput);
            setPivotTargetAngle(getPivotAngle());
        }
        pivotMotor.setPower(pivotOutput);

        if (!zeroing) {
            if (Math.abs(gamepad.right_stick_y) > 0.5) {
                extensionOutput = gamepad.right_stick_y > 0.5 ? 0.1 : -0.1;
                setExtensionTargetLength(getExtensionLength());
            }
            extensionMotorLeft.setPower(extensionOutput);
            extensionMotorRight.setPower(extensionOutput);
        } else {
            extensionMotorLeft.setPower(-0.5);
            extensionMotorRight.setPower(-0.5);
            if (extensionMotorLeft.getVelocity() == 0) {
                extensionMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setExtensionTargetLength(0);
                extensionMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                zeroing = false;
            }
        }
    }

    public double linearScaledFeedForward() {
        double min_extension = 0.0;
        double max_extension = 100.0;
        double min_feedforward = 0.05;
        double max_feedforward = 0.1;
        return (max_feedforward - min_feedforward) / (max_extension - min_extension) * (getExtensionLength() - min_extension) + min_feedforward;
    }

    // Setter Methods
    public void setPivotTargetAngle(double angle) {
        this.pivotTargetAngle = angle;
    }
    public void setExtensionTargetLength(double length) {
        this.extensionTargetLength = length;
    }
    public void zeroExtension(boolean zeroing) {
        this.zeroing = zeroing;
    }

    // Getter Methods
    public double getPivotAngle() {
        return pivotEncoder.getVoltage() / 3.2 * 360;
    }
    public double getExtensionLength() {
        return extensionMotorLeft.getCurrentPosition();
    }
    public double getExtensionVelocity() { return extensionMotorLeft.getVelocity(); }
    public double getPivotVelocity() {
        return pivotMotor.getVelocity();
    }
    public double getExtensionCurrent() {
        return extensionMotorLeft.getCurrent(CurrentUnit.AMPS);
    }
    public double getPivotCurrent() {
        return pivotMotor.getCurrent(CurrentUnit.AMPS);
    }
}
