package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
//import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class  StackingAlign implements Subsystem {
    private Robot robot;
    public static PIDCoefficients foundationDistanceCoefficients = new PIDCoefficients(-0.05,0,0);
    public static PIDCoefficients foundationAngleCoefficients = new PIDCoefficients(-1,0,0);
    public static PIDCoefficients StackCoefficients = new PIDCoefficients(0.03,0,0);
    public static double foundationAlignDistance = 6.5;
    public static double stackAlignDistance = 11;
    public static double stackAlignBound = 0.22;
    public static double stackAlignMin = 0.17;  //minimum power to drive robot
    public static final double TOF_SENSOR_DISTANCE = 10.3950008;
    public static final double TOF_ERR_MAX =  8.0;
    private PIDFController foundationDistanceController;
    private PIDFController foundationAngleController;
    private PIDFController stackAlignController;
    private int stage = 0;
    private static final double detectRange = 30;
    public  boolean do_align = false;
    public double verticalCorrection = 0.0;
    public double horizontalCorrection = 0.0;
    public double angelCorrection = 0.0;
    public double d1, d2, d3, d4 = 0;


    int count = 0;
    boolean skip_horizontal = false;

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
        stackAlignController = new  PIDFController(StackCoefficients);

        stackAlignController.setOutputBounds(-stackAlignBound, stackAlignBound);

        foundationDistanceController.setTargetPosition(foundationAlignDistance);
        foundationAngleController.setTargetPosition(0);
        stackAlignController.setTargetPosition(stackAlignDistance);

    }

    public void setDo_align(int stage) {
        this.stage = stage;

        do_align = !do_align;
        if (stage <= 1) {
             skip_horizontal = true;
        } else {
            skip_horizontal = false;
        }
    }

    private void foundationAlignUpdate() {

        d1 = TOF1.getDistance(DistanceUnit.CM) * 0.8179308772 + 0.444331014;
        d2 = TOF2.getDistance(DistanceUnit.CM) * 0.6343905382 + 2.814252421;

        double distance = (d1 + d2) / 2;
        double angle = Math.atan2(d1-d2, TOF_SENSOR_DISTANCE);
        verticalCorrection = foundationDistanceController.update(distance);
        angelCorrection  = foundationAngleController.update(angle);
        /*
        if (distance < 8  && stage > 1) {
            skip_horizontal = false;
        }*/

        /* horizontal adjust might introduce error in foundation,  so keeping working
         * till alignment has  done.  keep following line commented out for history
         * reason
         */
        /*
        if (Math.abs(foundationAngleController.getLastError()) < 5) {
            //foundationAngleController.reset();
            angelCorrection = 0;
        }
        if (Math.abs(foundationDistanceController.getLastError()) < 0.5) {
         //   foundationDistanceController.reset();
            verticalCorrection = 0;
        }*/

    }

    private void stackAlignupdate()
    {


        d3 = TOF3.getDistance(DistanceUnit.CM);
        d4 = TOF4.getDistance(DistanceUnit.CM);
        double distance = (d3 + d4) / 2;


        // check whether job is done
        if (d3 > detectRange && d4 > detectRange) {
            horizontalCorrection = 0;
            skip_horizontal = true;
            return;
        }

        if ( Math.abs(d3 - d4) < TOF_ERR_MAX ) {
            horizontalCorrection = 0;
            do_align = false;
            stackAlignController.reset();
            /* reset other PID too,  since adjust has done */
            foundationAngleController.reset();
            foundationDistanceController.reset();
        } else {
            horizontalCorrection = Math.abs(stackAlignController.update(distance));
        }

        if (horizontalCorrection < stackAlignMin ) {
            horizontalCorrection = stackAlignMin;
        }
         // PID lib might make mistake,  let's do our own work

        if (d3 > d4) {
            horizontalCorrection = - horizontalCorrection;
        }

    }


    @Override
    public void update() {
        if (do_align) {
            foundationAlignUpdate();
            stackAlignupdate();
        }

    }

    @Override
    public boolean isBusy() {
        return false;
    }
}
