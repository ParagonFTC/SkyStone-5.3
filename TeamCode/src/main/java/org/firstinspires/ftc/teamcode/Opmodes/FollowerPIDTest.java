package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp
public class FollowerPIDTest extends LinearOpMode {
    public static double DISTANCE = 48;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        robot.start();

        robot.drive.setPoseEstimate(new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0));

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            robot.drive.followTrajectorySync(
                    robot.drive.trajectoryBuilder()
                            .forward(DISTANCE)
                            .build()
            );
            robot.drive.turnSync(Math.toRadians(90));
        }
    }
}
