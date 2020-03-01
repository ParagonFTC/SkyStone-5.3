package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveWrapper;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.AutoTransitioner;
import org.firstinspires.ftc.teamcode.Vision.CustomSkystoneDetector;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public abstract class SkystoneAutoOpMode extends LinearOpMode {
    protected Robot robot;

    protected CustomSkystoneDetector detector;
    private OpenCvInternalCamera camera;

    public enum SkystonePosition {
        LEFT,
        CENTER,
        RIGHT
    }

    private SkystonePosition position;

    protected abstract SkystonePosition getSkystonePosition();
    protected abstract void setup();
    protected abstract void run(SkystonePosition position);
    @Override
    public final void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.start();

        robot.drive.setEncoderMode(true);
        robot.drive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MecanumDriveWrapper.MOTOR_VELO_PID);

        detector = new CustomSkystoneDetector();
        detector.useDefaults();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setFlashlightEnabled(true);
        camera.setRecordingHint(true);
        camera.setPipeline(detector);
        camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        setup();

        while (!isStarted() && !isStopRequested()) {
            position = getSkystonePosition();
            telemetry.addData("Skystone Position", position);
            telemetry.update();
        }

        camera.stopStreaming();

        //uncomment this line when auto is competition ready ;)
        //AutoTransitioner.transitionOnStop(this, "Teleop");

        waitForStart();

        if (isStopRequested()) return;

        run(position);
    }
}
