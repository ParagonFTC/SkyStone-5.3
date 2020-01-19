package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.RoadRunnerDriveTest;

@Config
@Autonomous
public class BlueAutoFoundation extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RoadRunnerDriveTest drive = new RoadRunnerDriveTest(hardwareMap);

        drive.disengageHooks();
        drive.armPusher();

        telemetry.addLine("initialization complete");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
/*
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(18, -30,Math.PI/2))
                        .build()
        );

        drive.disarmPusher();

        drive.setIntakePower(1);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(30,-56,Math.PI*3/4))
                        .build()
        );
        sleep(2000);

        drive.armPusher(); */

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .splineTo(new Pose2d(48,18,0))
                .build()
        );

        drive.engageHooks();
        sleep(1000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0,0,Math.PI/2))
                        .build()
        );

        drive.disengageHooks();
        sleep(1000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(12)
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0,-30,Math.PI/2))
                        .build()
        );
    }
}
