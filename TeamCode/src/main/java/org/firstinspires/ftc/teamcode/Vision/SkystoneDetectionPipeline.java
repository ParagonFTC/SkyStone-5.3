package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.*;
import org.opencv.imgproc.*;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * SkystoneDetectionPipeline class.
 *
 * <p>An OpenCV pipeline generated by GRIP.
 *
 * @author GRIP
 */
public class SkystoneDetectionPipeline extends OpenCvPipeline {

    //Outputs
    private Mat resizeImageOutput = new Mat();
    private Mat hsvThresholdOutput = new Mat();

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    /**
     * This is the primary method that runs the entire pipeline and updates the outputs.
     */
    public Mat processFrame(Mat source0) {
        // Step Resize_Image0:
        Mat resizeImageInput = source0;
        double resizeImageWidth = 700.0;
        double resizeImageHeight = 100.0;
        int resizeImageInterpolation = Imgproc.INTER_CUBIC;
        resizeImage(resizeImageInput, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, resizeImageOutput);

        // Step HSV_Threshold0:
        Mat hsvThresholdInput = resizeImageOutput;
        double[] hsvThresholdHue = {12.949640287769784, 180.0};
        double[] hsvThresholdSaturation = {0.0, 255.0};
        double[] hsvThresholdValue = {25.22482014388489, 255.0};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        return source0;
    }

    /**
     * This method is a generated getter for the output of a Resize_Image.
     * @return Mat output from Resize_Image.
     */
    public Mat resizeImageOutput() {
        return resizeImageOutput;
    }

    /**
     * This method is a generated getter for the output of a HSV_Threshold.
     * @return Mat output from HSV_Threshold.
     */
    public Mat hsvThresholdOutput() {
        return hsvThresholdOutput;
    }


    /**
     * Scales and image to an exact size.
     * @param input The image on which to perform the Resize.
     * @param width The width of the output in pixels.
     * @param height The height of the output in pixels.
     * @param interpolation The type of interpolation.
     * @param output The image in which to store the output.
     */
    private void resizeImage(Mat input, double width, double height,
                             int interpolation, Mat output) {
        Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
    }

    /**
     * Segment an image based on hue, saturation, and value ranges.
     *
     * @param input The image on which to perform the HSL threshold.
     * @param hue The min and max hue
     * @param sat The min and max saturation
     * @param val The min and max value
     * @param out The image in which to store the output.
     */
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }
}
