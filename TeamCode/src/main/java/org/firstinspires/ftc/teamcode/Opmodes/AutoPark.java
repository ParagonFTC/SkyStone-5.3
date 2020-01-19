package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.RoadRunnerDriveTest;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.StickyGamepad;

@Autonomous
public class AutoPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1);
        int delay = 0;

        robot.drive.disengageHooks();

        telemetry.addLine("initialization complete");
        telemetry.update();
        while (!isStarted()) {
            if (stickyGamepad1.dpad_up) delay ++;
            if (stickyGamepad1.dpad_down && delay != 0) delay --;
            telemetry.addData("delay", delay);
            telemetry.update();
        }
        waitForStart();

        if (isStopRequested()) return;

        sleep(delay * 1000);

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .forward(44)
                        .build()
        );
    }
}
