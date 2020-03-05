package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp
public class FoundationAlignTest extends LinearOpMode {
    public static PIDCoefficients foundationDistanceCoefficients = new PIDCoefficients(-0.05,0,0);
    public static PIDCoefficients foundationAngleCoefficients = new PIDCoefficients(-1,0,0);
    public static double foundationAlignDistance = 8;
    public static final double TOF_SENSOR_DISTANCE = 10.3950008;
    private PIDFController foundationDistanceController;
    private PIDFController foundationAngleController;

    //Function<Double, Double> TOF1Calibration = d1 -> 0.6657317711 * d1 + 2.33512549;
    //Function<Double, Double> TOF2Calibration = d2 -> 0.6343905382 * d2 + 2.814252421;
    @Override
    public void runOpMode() throws InterruptedException {
        Rev2mDistanceSensor TOF1 = hardwareMap.get(Rev2mDistanceSensor.class, "TOF1");
        Rev2mDistanceSensor TOF2 = hardwareMap.get(Rev2mDistanceSensor.class, "TOF2");
        Robot robot = new Robot(this);
        robot.start();

        foundationDistanceController = new PIDFController(foundationDistanceCoefficients);
        foundationAngleController = new PIDFController(foundationAngleCoefficients);

        waitForStart();
        if (isStopRequested()) return;

        foundationDistanceController.setTargetPosition(foundationAlignDistance);
        foundationAngleController.setTargetPosition(0);

        while (!isStopRequested()) {
            double d1 = TOF1.getDistance(DistanceUnit.CM) * 0.8179308772 + 0.444331014;
            double d2 = TOF2.getDistance(DistanceUnit.CM) * 0.6343905382 + 2.814252421;

            //double d1 = TOF1.getDistance(DistanceUnit.CM);
            //double d2 = TOF2.getDistance(DistanceUnit.CM);

            double distance = (d1 + d2) / 2;
            double angle = Math.atan2(d1-d2, TOF_SENSOR_DISTANCE);

            telemetry.addData("d1", d1);
            telemetry.addData("d2", d2);
            telemetry.addData("Distance", distance);
            telemetry.addData("Angle", angle);
            telemetry.update();

            double distanceCorrection = foundationDistanceController.update(distance);
            double angleCorrection = foundationAngleController.update(angle);
            robot.drive.setDrivePower(new Pose2d(distanceCorrection,0,angleCorrection));
        }
    }
}