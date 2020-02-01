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
    public static double LEFT_STONE_X = 42;

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
        robot.drive.setPoseEstimate(new Pose2d());
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
                                .splineTo(new Pose2d(-36,4,Math.toRadians(-30)))
                                .addMarker(() -> {robot.intake.setSpeed(0.8); return Unit.INSTANCE;})
                                .strafeTo(new Vector2d(-48,4))
                                .strafeTo(new Vector2d(-48,12))
                                .build()
                );
                robot.drive.setPoseEstimate(new Pose2d(-46,13,Math.toRadians(-55)));
                break;
            case RIGHT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-32,8,Math.toRadians(-30)))
                                .addMarker(() -> {robot.intake.setSpeed(0.8); return Unit.INSTANCE;})
                                .strafeTo(new Vector2d(-48,8))
                                .strafeTo(new Vector2d(-48,16))
                                .build()
                );
                robot.drive.setPoseEstimate(new Pose2d(-48,16,Math.toRadians(-55)));
                break;
            case CENTER:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-36,-2,Math.toRadians(-30)))
                                .addMarker(() -> {robot.intake.setSpeed(0.8); return Unit.INSTANCE;})
                                .strafeTo(new Vector2d(-48,-2))
                                .strafeTo(new Vector2d(-48,8))
                                .build()
                );
                robot.drive.setPoseEstimate(new Pose2d(-46,8,Math.toRadians(-55)));
                break;
        }
        sleep(500);

        robot.intake.setSpeed(0);
        robot.outtake.cycleWrist();
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .addMarker(1,() -> {robot.intake.setSpeed(-1); return Unit.INSTANCE;})
                        .addMarker(2, () -> {robot.intake.setSpeed(0); return Unit.INSTANCE;})
                        .splineTo(new Pose2d(-36,-64,Math.toRadians(-90)))
                        .splineTo(new Pose2d(-60,-105,Math.toRadians(-170)))
                        .forward(6)
                        .build()
        );
        robot.drive.engageHooks();
        robot.drive.setPoseEstimate(new Pose2d(-66,-105,Math.toRadians(-170)));
        sleep(500);
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .reverse()
                        .forward(12)
                        .splineTo(new Pose2d(-44,-65,Math.toRadians(-80)))
                        .build()
        );
        robot.drive.disengageHooks();
        robot.outtake.cycleWrist();
        sleep(100);
        robot.outtake.cycleWrist();
        sleep(1000);
        robot.outtake.cycleWrist();
        robot.drive.setPoseEstimate(new Pose2d(-40,-65,Math.toRadians(-90)));
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .forward(36)
                        .reverse()
                        .splineTo(new Pose2d(-60,-55,Math.toRadians(-90)))
                        .build()
        );
        robot.drive.setPoseEstimate(new Pose2d(-42,-55,Math.toRadians(-90)));
        /*
        switch (position) {
            case RIGHT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-41, -30, Math.toRadians(-90)))
                                .addMarker(() -> {
                                    robot.intake.setSpeed(0.8);
                                    return Unit.INSTANCE;
                                })
                                .splineTo(new Pose2d(-48, -20, Math.toRadians(-60)))
                                .strafeTo(new Vector2d(-52, -8))
                                .build()
                );
                robot.drive.setPoseEstimate(new Pose2d(-52, -4, Math.toRadians(-60)));
                break;
            case CENTER:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-41, 2, Math.toRadians(-90)))
                                .splineTo(new Pose2d(-52, 12, Math.toRadians(-60)))
                                .addMarker(() -> {
                                    robot.intake.setSpeed(0.8);
                                    return Unit.INSTANCE;
                                })
                                .strafeTo(new Vector2d(-52, 20))
                                .build()
                );
                robot.drive.setPoseEstimate(new Pose2d(-52, 24, Math.toRadians(-60)));
                break;
            case LEFT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-41, 10, Math.toRadians(-90)))
                                .splineTo(new Pose2d(-52, 20, Math.toRadians(-60)))
                                .addMarker(() -> {
                                    robot.intake.setSpeed(0.8);
                                    return Unit.INSTANCE;
                                })
                                .strafeTo(new Vector2d(-52, 28))
                                .build()
                );
                robot.drive.setPoseEstimate(new Pose2d(-52, 32, Math.toRadians(-60)));
                break;
        }

        sleep(500);

        robot.intake.setSpeed(0);
        robot.outtake.cycleWrist();

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .addMarker(1,() -> {robot.intake.setSpeed(-1); return Unit.INSTANCE;})
                        .addMarker(2, () -> {robot.intake.setSpeed(0); return Unit.INSTANCE;})
                        .splineTo(new Pose2d(-41,-16,Math.toRadians(-90)))
                        .splineTo(new Pose2d(-41,-88,Math.toRadians(-90)))
                        .build()
        );
        robot.drive.setPoseEstimate(new Pose2d(-41,-88,Math.toRadians(-90)));
        robot.outtake.deploy();
        sleep(1000);
        robot.outtake.cycleWrist();

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(-48,-48,Math.toRadians(-90)))
                        .build()
        ); */

    }
}
