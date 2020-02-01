package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import kotlin.Unit;

@Autonomous
public class BlueAuto extends SkystoneAutoOpMode{
    public static final double STONE_WIDTH = 49;
    // left and right are switched bc i'm lazy
    public static double RIGHT_STONE_X = 63;
    public static double CENTER_STONE_X = 125;
    public static double LEFT_STONE_X = 184;

    @Override
    protected SkystoneAutoOpMode.SkystonePosition getSkystonePosition() {
        double xPos = detector.getScreenPosition().x;
        if (Math.abs(RIGHT_STONE_X - xPos) < STONE_WIDTH/2) return SkystoneAutoOpMode.SkystonePosition.RIGHT;
        else if (Math.abs(CENTER_STONE_X - xPos) < STONE_WIDTH/2) return SkystoneAutoOpMode.SkystonePosition.CENTER;
        else if (Math.abs(LEFT_STONE_X - xPos) < STONE_WIDTH/2) return SkystoneAutoOpMode.SkystonePosition.LEFT;
        return SkystoneAutoOpMode.SkystonePosition.RIGHT;
    }

    @Override
    protected void setup() {
        robot.drive.disengageHooks();
        robot.outtake.cycleWrist();
        robot.drive.setPoseEstimate(new Pose2d());
    }

    @Override
    protected void run(SkystoneAutoOpMode.SkystonePosition position) {
        //for some reason position estimation is bad so I manually set the position of the robot
        switch (position) {
            case LEFT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-36,-4,Math.toRadians(30)))
                                .addMarker(() -> {robot.intake.setSpeed(0.8); return Unit.INSTANCE;})
                                .strafeTo(new Vector2d(-48,-4))
                                .strafeTo(new Vector2d(-48,-12))
                                .build()
                );
                robot.drive.setPoseEstimate(new Pose2d(-46,-13,Math.toRadians(55)));
                break;
            case RIGHT:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-32,-8,Math.toRadians(30)))
                                .addMarker(() -> {robot.intake.setSpeed(0.8); return Unit.INSTANCE;})
                                .strafeTo(new Vector2d(-48,-8))
                                .strafeTo(new Vector2d(-48,-16))
                                .build()
                );
                robot.drive.setPoseEstimate(new Pose2d(-48,-16,Math.toRadians(55)));
                break;
            case CENTER:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(-36,2,Math.toRadians(30)))
                                .addMarker(() -> {robot.intake.setSpeed(0.8); return Unit.INSTANCE;})
                                .strafeTo(new Vector2d(-48,2))
                                .strafeTo(new Vector2d(-48,-8))
                                .build()
                );
                robot.drive.setPoseEstimate(new Pose2d(-46,-8,Math.toRadians(55)));
                break;
        }
        sleep(500);

        robot.intake.setSpeed(0);
        robot.outtake.cycleWrist();
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .addMarker(1,() -> {robot.intake.setSpeed(-1); return Unit.INSTANCE;})
                        .addMarker(2, () -> {robot.intake.setSpeed(0); return Unit.INSTANCE;})
                        .splineTo(new Pose2d(-36,64,Math.toRadians(90)))
                        .splineTo(new Pose2d(-60,105,Math.toRadians(170)))
                        .forward(6)
                        .build()
        );
        robot.drive.engageHooks();
        robot.drive.setPoseEstimate(new Pose2d(-66,105,Math.toRadians(170)));
        sleep(500);
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .reverse()
                        .forward(12)
                        .splineTo(new Pose2d(-44,65,Math.toRadians(80)))
                        .build()
        );
        robot.drive.disengageHooks();
        robot.outtake.cycleWrist();
        sleep(100);
        robot.outtake.cycleWrist();
        sleep(1000);
        robot.outtake.cycleWrist();
        robot.drive.setPoseEstimate(new Pose2d(-40,65,Math.toRadians(90)));
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .forward(36)
                        .reverse()
                        .splineTo(new Pose2d(-60,55,Math.toRadians(90)))
                        .build()
        );
        robot.drive.setPoseEstimate(new Pose2d(-42,55,Math.toRadians(90)));
    }
}
