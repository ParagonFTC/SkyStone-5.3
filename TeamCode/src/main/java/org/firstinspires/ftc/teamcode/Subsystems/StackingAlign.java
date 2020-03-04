package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class  StackingAlign implements Subsystem {
    private Robot robot;
    public static PIDCoefficients foundationDistanceCoefficients = new PIDCoefficients(-0.05,0,0);
    public static PIDCoefficients foundationAngleCoefficients = new PIDCoefficients(-1,0,0);
    public static double foundationAlignDistance = 6.5;
    public static final double TOF_SENSOR_DISTANCE = 10.3950008;
    public static final double TOF_ERR_MAX =  6.0;
    private PIDFController foundationDistanceController;
    private PIDFController foundationAngleController;
    public  boolean do_align = false;
    public double verticalCorrection = 0;
    public double horizintalCorrection = 0;
    public double angelCorrection = 0;
    public double d3, d4;
    NanoClock clock;
    double startTimestamp = 0;
    int count = 0;
    boolean skip_horizontal = true;

    Rev2mDistanceSensor TOF1;
    Rev2mDistanceSensor TOF2;
    Rev2mDistanceSensor TOF3;
    Rev2mDistanceSensor TOF4;

    public  StackingAlign (HardwareMap hardwareMap) {

        TOF1 = hardwareMap.get(Rev2mDistanceSensor.class, "TOF1");
        TOF2 = hardwareMap.get(Rev2mDistanceSensor.class, "TOF2");
        TOF3 = hardwareMap.get(Rev2mDistanceSensor.class, "TOF3");
        TOF4 = hardwareMap.get(Rev2mDistanceSensor.class, "TOF4");
        foundationDistanceController = new PIDFController(foundationDistanceCoefficients);
        foundationAngleController = new PIDFController(foundationAngleCoefficients);

        foundationDistanceController.setTargetPosition(foundationAlignDistance);
        foundationAngleController.setTargetPosition(0);

    }

    public void setDo_align(int stage) {
        do_align = !do_align;
        if (stage == 1) {
            skip_horizontal = true;
        } else {
            skip_horizontal = true;
        }
       //startTimestamp = clock.seconds();
    }

    private void verticalupdate() {

        double d1 = TOF1.getDistance(DistanceUnit.CM) * 0.6657317711 + 2.33512549;
        double d2 = TOF2.getDistance(DistanceUnit.CM) * 0.6343905382 + 2.814252421;

        double distance = (d1 + d2) / 2;
        double angle = Math.atan2(d1-d2, TOF_SENSOR_DISTANCE);
        verticalCorrection = foundationDistanceController.update(distance);
        angelCorrection  = foundationAngleController.update(angle);
        // robot.drive.setDrivePower(new Pose2d(distanceCorrection,0,angleCorrection));
    }

    private void horizontalupdate()
    {

        if (skip_horizontal) {
            horizintalCorrection = 0;

        }
        d3 = TOF3.getDistance(DistanceUnit.CM);
        d4 = TOF4.getDistance(DistanceUnit.CM);

        if (d3 > 30 && d4 > 30 ) {
            horizintalCorrection = 0;
        }

        if (Math.abs(d3 - d4) < TOF_ERR_MAX ) {
            horizintalCorrection = 0;
            do_align = false;
        }
        if (d3 > d4) {
            horizintalCorrection = -0.25;
        } else {
            horizintalCorrection = 0.25;
        }
    }


    @Override
    public void update() {
        if (do_align) {
            if (count > 45) {
                do_align = false;
                count = 0;
            }
            verticalupdate();
            horizontalupdate();
            count ++;
        }

    }

    @Override
    public boolean isBusy() {
        return false;
    }
}
