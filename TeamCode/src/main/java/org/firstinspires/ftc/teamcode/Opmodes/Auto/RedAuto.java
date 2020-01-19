package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import kotlin.Unit;

@Config
@Autonomous
public class RedAuto extends SkystoneAutoOpMode {
    public static final double STONE_WIDTH = 49;
    public static double RIGHT_STONE_X = 158;
    public static double CENTER_STONE_X = 98;
    public static double LEFT_STONE_X = 38;

    @Override
    protected SkystonePosition getSkystonePosition() {
        double xPos = detector.getScreenPosition().x;
        if (Math.abs(RIGHT_STONE_X - xPos) < STONE_WIDTH/2) return SkystonePosition.RIGHT;
        else if (Math.abs(CENTER_STONE_X - xPos) < STONE_WIDTH/2) return SkystonePosition.CENTER;
        else if (Math.abs(LEFT_STONE_X - xPos) < STONE_WIDTH/2) return SkystonePosition.LEFT;
        return SkystonePosition.RIGHT;
    }

    @Override
    protected void setup() {
        robot.drive.disengageHooks();
        robot.outtake.cycleWrist();
        //robot.outtake.setWristPosition(0.2);
    }

    @Override
    protected void run(SkystonePosition position) {
        //for some reason position estimation is bad so I manually set the position of the robot
        switch (position) {
            case LEFT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-40,-4,Math.toRadians(-45)))
                                .addMarker(() -> {robot.intake.toggleIntake(); return Unit.INSTANCE;})
                                .splineTo(new Pose2d(-56,12,Math.toRadians(-45)))
                                .build()
                );
                robot.drive.setPoseEstimate(new Pose2d(-56,12,Math.toRadians(-45)));
                break;
            case RIGHT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-44,-16,Math.toRadians(-60)))
                                .addMarker(() -> {robot.intake.toggleIntake(); return Unit.INSTANCE;})
                                .strafeTo(new Vector2d(-64,-2))
                                .build()
                );
                robot.drive.setPoseEstimate(new Pose2d(-64,-2,Math.toRadians(-60)));
                break;
            case CENTER:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-44,-16,Math.toRadians(-55)))
                                .addMarker(() -> {robot.intake.toggleIntake(); return Unit.INSTANCE;})
                                .strafeTo(new Vector2d(-60,-16))
                                .strafeTo(new Vector2d(-60,0))
                                .build()
                );
                robot.drive.setPoseEstimate(new Pose2d(-60,0,Math.toRadians(-55)));
                break;
        }
        sleep(500);
        robot.intake.setSpeed(0);
        robot.outtake.cycleWrist();
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-36,-28,Math.toRadians(-90)))
                        .strafeTo(new Vector2d(-40,-68))
                        .splineTo(new Pose2d(-72,-96,Math.toRadians(-170)))
                        .build()
        );
        robot.drive.engageHooks();
        robot.drive.setPoseEstimate(new Pose2d(-72,-96,Math.toRadians(-170)));
        sleep(500);
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(-36,-60,Math.toRadians(-80)))
                        .setReversed(false)
                        .splineTo(new Pose2d(-24,-108,Math.toRadians(-60)))
                        .build()
        );
        robot.drive.setPoseEstimate(new Pose2d(-36,-108,Math.toRadians(-110)));
        robot.drive.disengageHooks();
        robot.outtake.cycleWrist();
        robot.outtake.cycleWrist();
        sleep(1000);
        robot.outtake.cycleWrist();
        robot.outtake.liftPositionDown();
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .reverse()
                        .forward(56)
                        .build()
        );
    }
}
