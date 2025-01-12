// Archive

package org.firstinspires.ftc.teamcode.config.subsystems.Folder;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class pivotLiftArchive {
    private Telemetry telemetry;
    public DcMotorEx rightLift, leftLift, pivot;
    public AnalogInput pivotEncoder;
    public boolean manualLift = false;
    public boolean manualPivot = false;
    public boolean hang =  false;
    public int pos, bottom;
    public int pivotStart = 10;
    public PIDController liftPID, pivotPID;
    public static int liftTarget=0, pivotTarget=0;
    public static double liftKP = 0.003, liftKI = 0.0, liftKD = 0, liftKF = 0.15;
    public static double pivotKP = 0.02, pivotKI = 0.0, pivotKD = 0.001, pivotKf;
    public static double min_extension = 0.0;
    public static double max_extension = 35000;
    public static double min_feedforward = 0.05;
    public static double max_feedforward = 0.1;

    public pivotLiftArchive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivot_enc");

        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        liftPID = new PIDController(liftKP, liftKI, liftKD);
        pivotPID = new PIDController(pivotKP, pivotKI, pivotKD);
    }

    public void updateLiftPIDF() {
        if (!manualLift) {
            liftPID.setPID(liftKP, liftKI, liftKD);

            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            double pid = liftPID.calculate(getLiftPos(), liftTarget);
            double ff = liftKF * Math.sin(Math.toRadians(getPivotAngle()));
            double power = (pid + ff) / 12.75 * 12;

            rightLift.setPower(-1 * power);
            leftLift.setPower(-1 * power);

            telemetry.addData("lift pos", getLiftPos());
            telemetry.addData("lift target", liftTarget);
        }
    }

    public void updatePivotPIDF() {
        if (!manualPivot) {
            double ff;
            pivotPID.setPID(pivotKP, pivotKI, pivotKD);
            double pid = pivotPID.calculate(getPivotAngle(), pivotTarget);
            if (getPivotAngle() > 90) {
                ff = linearlyScaledFF() * Math.cos(Math.toRadians(getPivotAngle()-90));
            } else {
                ff = -1 * linearlyScaledFF() * Math.cos(Math.toRadians(90-getPivotAngle()));
            }
            pivotKf = ff;
            double power = pid + ff;

            pivot.setPower(power);

            telemetry.addData("pivot angle", getPivotAngle());
            telemetry.addData("pivot target", pivotTarget);
        }
    }

    public void manualLift(double n) {
        manualLift = true;

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(hang) {
            n = -0.75;
        }

        rightLift.setPower(n);
        leftLift.setPower(n);
        liftTarget = getLiftPos();

        if (rightLift.getCurrent(CurrentUnit.AMPS) > 5 && !hang) {
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftTarget = 0;
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void manualPivot(double n) {
        manualPivot = true;
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pivot.setPower(n);
    }

    // Util
    public void liftTargetCurrent() {
        liftTarget = getLiftPos();
    }

    public double getLiftTarget() {
        return liftTarget;
    }

    public double getPivotTarget() {
        return pivotTarget;
    }

    public void setLiftTarget(int target) {
        liftTarget = target;
    }

    public void setPivotTarget(int target) {
        pivotTarget = target;
    }

    public void addToLiftTarget(int target) {
        liftTarget += target;
    }

    public void addToPivotTarget(int target) {
        pivotTarget += target;
    }

    public int getLiftPos() {
        pos = rightLift.getCurrentPosition() - bottom;
        return pos;
    }

    public int getPivotAngle() {
        // round to the nearest degree
        return (int) Math.round(pivotEncoder.getVoltage() / 3.2 * 360) - 185;
    }


    public double linearlyScaledFF() {
        return (max_feedforward - min_feedforward) / (max_extension - min_extension) * (getLiftPos() - min_extension) + min_feedforward;
    }

    // OpMode
    public void init() {
        liftPID.setPID(liftKP, liftKI, liftKD);
        bottom = getLiftPos();
        pivotPID.setPID(pivotKP, pivotKI, pivotKD);
    }

    public void start() {
        liftTarget = bottom;
        pivotTarget = pivotStart;
    }


    // Presets


}
