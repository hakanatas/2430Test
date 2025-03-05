package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class limelightOPMode extends LinearOpMode {
    private OpenCvWebcam camera;
    private limelight pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use an external webcam with the name configured in the Robot Controller
        String webcamName = "Webcam 1"; // Change this to the name you set in the configuration
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        pipeline = new limelight("red"); // Change "red" to "blue" or "yellow" if needed
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("Camera Stream Ready - Check DS Menu");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            Point blockCenter = pipeline.getBlockCenter();
            double blockAngle = pipeline.getBlockAngle();
            double blockY = pipeline.getBlockY();

            boolean detected = (blockCenter.x != -1 && blockCenter.y != -1 && blockY > 0);

            telemetry.addData("Block Detected", detected ? "Yes" : "No");
            if (detected) {
                telemetry.addData("Block X (Translational)", blockCenter.x);
                telemetry.addData("Block Y (Forward/Backward)", blockY);
                telemetry.addData("Block Angle", blockAngle);
            }
            telemetry.update();
        }

        camera.closeCameraDevice();
    }
}