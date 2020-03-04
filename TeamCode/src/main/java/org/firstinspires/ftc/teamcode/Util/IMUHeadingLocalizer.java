package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.jetbrains.annotations.NotNull;

public class IMUHeadingLocalizer implements Localizer {
    private Localizer localizer;
    private BNO055IMU imu;
    private double headingCorrection;
    public IMUHeadingLocalizer(Localizer localizer, BNO055IMU imu) {
        this.localizer = localizer;
        this.imu = imu;
    }
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        double x = localizer.getPoseEstimate().getX();
        double y = localizer.getPoseEstimate().getY();
        double heading = Angle.norm(imu.getAngularOrientation().firstAngle - headingCorrection);
        return new Pose2d(x,y,heading);
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        localizer.setPoseEstimate(pose2d);
        headingCorrection = Angle.norm(imu.getAngularOrientation().firstAngle - pose2d.getHeading());
    }

    @Override
    public void update() {
        localizer.update();
    }
}
