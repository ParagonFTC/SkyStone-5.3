package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class  StackingAlign implements Subsystem {
    private Robot robot;
    public static PIDCoefficients foundationDistanceCoefficients = new PIDCoefficients(-0.1,0,0); //0.05
    public static PIDCoefficients foundationAngleCoefficients = new PIDCoefficients(-2,0,0); //-1.5, 0, -0.1
    public static PIDCoefficients StackCoefficients = new PIDCoefficients(0.5,0,0);   // 0.1
    public static double foundationAlignDistance = 6.8;
    public static double stackAlignDistance = 11;
    public static double stackAlignMin = 0.1;  //minimum power to drive robot //0.17
    public static final double TOF_SENSOR_DISTANCE = 10.3950008;
    public static final double TOF_ERR_MAX =  4.0;

    public static final double STACK_ALIGN_OUT_BOUND  = 0.24; // 0.22 is a good value,  can we try higher value
    public static final double TOF_DISTANCE_OUT_BOUND = 3;
    public static final double TOF_ANGLE_OUT_BOUND = 15.0 ;//10
    public static final double STACK_ALIGN_IN_BOUND  = 30;
    public static final double TOF_DISTANCE_IN_BOUND = 40;
    public static final double TOF_ANGLE_IN_BOUND = 30 ;
    public static final double TOF_DIST_MIN = 0.12;
    public static final double TOF_ANGLE_MIN = 0.1;

    private PIDFController foundationDistanceController;
    private PIDFController foundationAngleController;
    private PIDFController stackAlignController;
    private static final double horizontalDetectRange = 20;

    private static final double foundationDetectRange = 50;
    private static final double foundationDetectDIFF = 15;
    public  boolean do_align = false;
    public boolean do_stackalign = true;
    public boolean do_basealign_dist = true;
    public boolean do_basealign_angle=  true;
    public double verticalCorrection = 0.0;
    public double horizontalCorrection = 0.0;
    public double angelCorrection = 0.0;
    public double d1, d2, d3, d4 = 0;


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

        stackAlignController.setInputBounds(-STACK_ALIGN_IN_BOUND, STACK_ALIGN_IN_BOUND);
        foundationDistanceController.setInputBounds(-TOF_DISTANCE_IN_BOUND, TOF_DISTANCE_IN_BOUND);
        foundationAngleController.setInputBounds(-TOF_ANGLE_IN_BOUND, TOF_ANGLE_OUT_BOUND);

        stackAlignController.setOutputBounds(-STACK_ALIGN_OUT_BOUND, STACK_ALIGN_OUT_BOUND);
        foundationDistanceController.setOutputBounds(-TOF_ANGLE_OUT_BOUND, TOF_DISTANCE_OUT_BOUND);
        foundationAngleController.setOutputBounds(-TOF_ANGLE_OUT_BOUND, TOF_ANGLE_OUT_BOUND);

        foundationDistanceController.setTargetPosition(foundationAlignDistance);
        foundationAngleController.setTargetPosition(0);
        stackAlignController.setTargetPosition(stackAlignDistance);
    }

    public void setDo_align(int stage) {

        do_align = !do_align;
        do_stackalign =true;
        do_basealign_angle = true;
        do_basealign_dist = true;

        if (stage <= 1) {
             do_stackalign = false;
        }
    }

    private void foundationAlignUpdate() {
        if (do_basealign_angle == false && do_basealign_dist == false) {
            verticalCorrection = 0 ;
            angelCorrection = 0;
            return;
        }

        d1 = TOF1.getDistance(DistanceUnit.CM) * 0.728121473 + 1.48048505;
        d2 = TOF2.getDistance(DistanceUnit.CM) * 0.6343905382 + 2.814252421;

        // Limit distance adjust range
        if (Math.abs(d1 - d2) > foundationDetectDIFF || d1 >  foundationDetectRange || d2 > foundationDetectRange) {
            verticalCorrection = 0 ;
            angelCorrection = 0;
            //do_basealign_dist = false;
            return;
        }
        // Calculate measurements
        double distance = (d1 + d2) / 2;
        double angle = Math.atan2(d1-d2, TOF_SENSOR_DISTANCE);

        // Limit angle range
        if ( Math.abs(angle) > 30 ) {

            do_basealign_angle = false;
            verticalCorrection = 0;
            angelCorrection = 0;
            return;
        }

        verticalCorrection = foundationDistanceController.update(distance);
        angelCorrection  = foundationAngleController.update(angle);
/*
        if (Math.abs(verticalCorrection) < TOF_DIST_MIN) {
            verticalCorrection =  TOF_DIST_MIN;
        }


       if (distance <  foundationAlignDistance) {
           verticalCorrection = - verticalCorrection;
       }
        if (Math.abs(angelCorrection) < TOF_ANGLE_MIN) {
            angelCorrection =  TOF_ANGLE_MIN;
        }

        if (angle <  0 ) {
            angelCorrection = -angelCorrection;
        }
*/
        // Final error exit condition
        if (Math.abs(foundationAngleController.getLastError()) < 2) {
            //foundationAngleController.reset();
           // do_basealign_angle = false;
        }
        if (Math.abs(foundationDistanceController.getLastError()) < 0.5) {
           // do_basealign_dist = false;
        }

    }

    private void stackAlignUpdate()
    {

        if (do_stackalign == false )  {
            horizontalCorrection = 0;
            return;
        }

        d3 = TOF3.getDistance(DistanceUnit.CM);
        d4 = TOF4.getDistance(DistanceUnit.CM);
        double distance = (d3 + d4) / 2;


        // check whether job is done
        if (d3 > horizontalDetectRange && d4 > horizontalDetectRange) {
            horizontalCorrection = 0;
            return;
        }

        if ( Math.abs(distance - stackAlignDistance) < TOF_ERR_MAX ) {
            horizontalCorrection = 0;
            do_stackalign = false;
            stackAlignController.reset();
            /* reset other PID too,  since adjust has done */

           //foundationAngleController.reset();
           //foundationDistanceController.reset();
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

    public void testalign() {
        foundationAlignUpdate();
        stackAlignUpdate();
    }


    @Override
    public void update() {
        foundationAlignUpdate();
        stackAlignUpdate();
    }

    @Override
    public boolean isBusy() {
        return false;
    }
}
