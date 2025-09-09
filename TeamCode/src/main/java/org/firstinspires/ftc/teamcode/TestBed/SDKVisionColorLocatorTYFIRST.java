/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.TestBed;

import android.util.Size;

import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Core;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/*
 * This OpMode illustrates how to use a video source (camera) to locate specifically colored regions
 *
 * Unlike a "color sensor" which determines the color of an object in the field of view, this "color locator"
 * will search the Region Of Interest (ROI) in a camera image, and find any "blobs" of color that match the requested color range.
 * These blobs can be further filtered and sorted to find the one most likely to be the item the user is looking for.
 *
 * To perform this function, a VisionPortal runs a ColorBlobLocatorProcessor process.
 *   The ColorBlobLocatorProcessor process is created first, and then the VisionPortal is built to use this process.
 *   The ColorBlobLocatorProcessor analyses the ROI and locates pixels that match the ColorRange to form a "mask".
 *   The matching pixels are then collected into contiguous "blobs" of pixels.  The outer boundaries of these blobs are called its "contour".
 *   For each blob, the process then creates the smallest possible rectangle "boxFit" that will fully encase the contour.
 *   The user can then call getBlobs() to retrieve the list of Blobs, where each Blob contains the contour and the boxFit data.
 *   Note: The default sort order for Blobs is ContourArea, in descending order, so the biggest contours are listed first.
 *
 * To aid the user, a colored boxFit rectangle is drawn on the camera preview to show the location of each Blob
 * The original Blob contour can also be added to the preview.  This is helpful when configuring the ColorBlobLocatorProcessor parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@TeleOp(name = "Concept: Vision Color-Locator", group = "Concept")
public class SDKVisionColorLocatorTYFIRST extends LinearOpMode
{


    int desiredTagID;

    public String currentColor = "blue";

    double cx;

    boolean PIDActive;

    DcMotor mBL, mBR, mFL, mFR;

    Servo LED;

    double color = .5;

    double FB, LR, Rotation;

    PDController controller;
    public double p = .001, d = .0005;

    boolean targetFound = false;

    AprilTagDetection detectedTag;

    int Rr=140, Rg=34, Rb=22;
    int Br=39, Bg=50, Bb=110;
    int Yr=175, Yg=125, Yb=49;

    int error=25;

    static int imgHeight = 896;
    static int imgWidth = 1600;

    //private OpenCvCamera controlHubCam;

    List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();

    public static double calculateScore(ColorBlobLocatorProcessor.Blob c1)
    {
                double score = c1.getDensity()*.5 +(.5*(c1.getContourArea()/( 250000)));

                return score;
            }

    public  class CannyEdgeProcessor implements VisionProcessor {
        private Mat gray = new Mat();
        private Mat edges = new Mat();
        private Mat edgesColor = new Mat();
        private Mat overlay = new Mat();

        public void init(int width, int height, CameraCalibration calibration) {
            // Nothing special to initialize here

        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // Convert to grayscale
            Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGBA2GRAY);

            // Run Canny edge detection
            Imgproc.Canny(gray, edges, 100, 200);

            // Convert edges to 4-channel RGBA
            Imgproc.cvtColor(edges, edgesColor, Imgproc.COLOR_GRAY2RGBA);

            // ✅ Overlay edges (white edges) onto original frame
            Core.addWeighted(frame, 1.0, edgesColor, 1.0, 0.0, frame);

            return null;  // no object result, just visual modification
        }

        @Override
        public void onDrawFrame(android.graphics.Canvas canvas,
                                int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx,
                                float scaleCanvasDensity,
                                Object userContext) {
            // Nothing needed here unless you want extra debug drawings
        }
    }



    public static void sortByScore(SortOrder sortOrder, List<ColorBlobLocatorProcessor.Blob> blobs)
    {
        blobs.sort(new Comparator<ColorBlobLocatorProcessor.Blob>()
        {
            public int compare(ColorBlobLocatorProcessor.Blob c1, ColorBlobLocatorProcessor.Blob c2)
            {
                int tmp = (int)Math.signum(calculateScore(c2) - calculateScore(c1));

                if (sortOrder == SortOrder.ASCENDING)
                {
                    tmp = -tmp;
                }

                return tmp;
            }
        });
    }

    @Override
    public void runOpMode()
    {

        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

        mBR.setDirection(DcMotorSimple.Direction.REVERSE);
        mFR.setDirection(DcMotorSimple.Direction.REVERSE);

        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LED = hardwareMap.get(Servo.class,"LED");

        controller = new PDController(p, d);

        /* Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
         * - Specify the color range you are looking for.  You can use a predefined color, or create you own color range
         *     .setTargetColorRange(ColorRange.BLUE)                      // use a predefined color match
         *       Available predefined colors are: RED, BLUE YELLOW GREEN
         *     .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,      // or define your own color match
         *                                           new Scalar( 32, 176,  0),
         *                                           new Scalar(255, 255, 132)))
         *
         * - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
         *     This can be the entire frame, or a sub-region defined using:
         *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
         *     Use one form of the ImageRegion class to define the ROI.
         *         ImageRegion.entireFrame()
         *         ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixel square near the upper left corner
         *         ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5)  50% width/height square centered on screen
         *
         * - Define which contours are included.
         *     You can get ALL the contours, or you can skip any contours that are completely inside another contour.
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)  // return all contours
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)            // exclude contours inside other contours
         *        note: EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up areas of solid color.
         *
         * - turn the display of contours ON or OFF.  Turning this on helps debugging but takes up valuable CPU time.
         *        .setDrawContours(true)
         *
         * - include any pre-processing of the image or mask before looking for Blobs.
         *     There are some extra processing you can include to improve the formation of blobs.  Using these features requires
         *     an understanding of how they may effect the final blobs.  The "pixels" argument sets the NxN kernel size.
         *        .setBlurSize(int pixels)    Blurring an image helps to provide a smooth color transition between objects, and smoother contours.
         *                                    The higher the number of pixels, the more blurred the image becomes.
         *                                    Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
         *                                    Blurring too much may hide smaller features.  A "pixels" size of 5 is good for a 320x240 image.
         *        .setErodeSize(int pixels)   Erosion removes floating pixels and thin lines so that only substantive objects remain.
         *                                    Erosion can grow holes inside regions, and also shrink objects.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         *        .setDilateSize(int pixels)  Dilation makes objects more visible by filling in small holes, making lines appear thicker,
         *                                    and making filled shapes appear larger. Dilation is useful for joining broken parts of an
         *                                    object, such as when removing noise from an image.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         */



        ColorBlobLocatorProcessor blueLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(//ColorRange.BLUE
                new ColorRange(
                    ColorSpace.YCrCb,
                        new Scalar( 16,   0, 145),
                        new Scalar(255, 130, 255))
                )         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(20)                               // Smooth the transitions between different colors in image
                .setErodeSize(10)
                .build();

        AprilTagProcessor aTagP = new AprilTagProcessor.Builder().build();

        ColorBlobLocatorProcessor redLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(//ColorRange.RED
                        new ColorRange(ColorSpace.YCrCb,
                                new Scalar( 32, 166,  0),
                                new Scalar(255, 255, 132)
                        )
                )         // use a predefined red color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(20)                               // Smooth the transitions between different colors in image
                .setErodeSize(10)                               // Smooth the transitions between different colors in image
                .build();

        ColorBlobLocatorProcessor yellowLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(//ColorRange.YELLOW
                        new ColorRange(ColorSpace.YCrCb,
                                new Scalar(100 , 128,   0),
                                new Scalar(255, 180, 100)
                        )
                )
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(20)                               // Smooth the transitions between different colors in image
                .setErodeSize(10)                               // Smooth the transitions between different colors in image
                .build();


        /*
         * Build a vision portal to run the Color Locator process.
         *
         *  - Add the colorLocator process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution that is
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessors(blueLocator, aTagP, redLocator, yellowLocator, new CannyEdgeProcessor())
                .setCameraResolution(new Size(imgWidth, imgHeight))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();


        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.




        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit())
        {

            LED.setPosition(color);

            //portal.stopStreaming();
            //portal.resumeStreaming();

            if(gamepad1.a){
                portal.setProcessorEnabled(aTagP, true);
                portal.setProcessorEnabled(blueLocator, false);
                portal.setProcessorEnabled(redLocator, false);
                portal.setProcessorEnabled(yellowLocator, false);
            }
            else if(gamepad1.b){
                portal.setProcessorEnabled(aTagP, false);
                portal.setProcessorEnabled(blueLocator, false);
                portal.setProcessorEnabled(redLocator, true);
                portal.setProcessorEnabled(yellowLocator, false);

                currentColor = "red";


            }
            else if (gamepad1.x){
                portal.setProcessorEnabled(aTagP, false);
                portal.setProcessorEnabled(blueLocator, true);
                portal.setProcessorEnabled(redLocator, false);
                portal.setProcessorEnabled(yellowLocator, false);

                currentColor = "blue";


            }
            else if (gamepad1.y){
                portal.setProcessorEnabled(aTagP, false);
                portal.setProcessorEnabled(blueLocator, false);
                portal.setProcessorEnabled(redLocator, false);
                portal.setProcessorEnabled(yellowLocator, true);

                currentColor = "yellow";
            }
            else if (gamepad1.dpad_down){
                portal.setProcessorEnabled(new CannyEdgeProcessor(), false);
            }
            else if (gamepad1.dpad_up){
                portal.setProcessorEnabled(new CannyEdgeProcessor(), true);
            }

            ColorBlobLocatorProcessor.Util.filterByArea(1000, 400000, blobs);// filter out very small blobs.
            if (currentColor == "red"){
                blobs = redLocator.getBlobs();
            }
            else if (currentColor == "blue"){
                blobs = blueLocator.getBlobs();
            }
            else if (currentColor == "yellow"){
                blobs = yellowLocator.getBlobs();
            }

            telemetry.addData("preview on/off", "... Camera Stream\n");

            // Read the current list




            SDKVisionColorLocatorTYFIRST.sortByScore(SortOrder.DESCENDING, blobs);



            List<AprilTagDetection>currentDetections = aTagP.getDetections();




            if(gamepad1.dpad_up){
                desiredTagID = 5;
            }
            else if(gamepad1.dpad_left){
                desiredTagID = 4;
            }
            else if(gamepad1.dpad_right){
                desiredTagID = 6;
            }
            else if (gamepad1.dpad_down){
                desiredTagID = -1;
            }

            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
              //  if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (/*(desiredTagID < 0) ||*/ (detection.id == desiredTagID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        detectedTag = detection;
                        break;  // don't look any further.
                    } else {
                        targetFound = false;
                        // This tag is in the library, but we do not want to track it right now.
//                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
              //  } else {

                    // This tag is NOT in the library, so we don't have enough information to track to it.
//                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
             //   }
            }



            /*
             * The list of Blobs can be filtered to remove unwanted Blobs.
             *   Note:  All contours will be still displayed on the Stream Preview, but only those that satisfy the filter
             *          conditions will remain in the current list of "blobs".  Multiple filters may be used.
             *
             * Use any of the following filters.
             *
             * ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);
             *   A Blob's area is the number of pixels contained within the Contour.  Filter out any that are too big or small.
             *   Start with a large range and then refine the range based on the likely size of the desired object in the viewfinder.
             *
             * ColorBlobLocatorProcessor.Util.filterByDensity(minDensity, maxDensity, blobs);
             *   A blob's density is an indication of how "full" the contour is.
             *   If you put a rubber band around the contour you would get the "Convex Hull" of the contour.
             *   The density is the ratio of Contour-area to Convex Hull-area.
             *
             * ColorBlobLocatorProcessor.Util.filterByAspectRatio(minAspect, maxAspect, blobs);
             *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
             *   A perfect Square has an aspect ratio of 1.  All others are > 1
             */

            //ColorBlobLocatorProcessor.Util.filterByAspectRatio(2,2.6,blobs);

            /*
             * The list of Blobs can be sorted using the same Blob attributes as listed above.
             * No more than one sort call should be made.  Sorting can use ascending or descending order.
             *     ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);      // Default
             *     ColorBlobLocatorProcessor.Util.sortByDensity(SortOrder.DESCENDING, blobs);
             *     ColorBlobLocatorProcessor.Util.sortByAspectRatio(SortOrder.DESCENDING, blobs);
             */




            telemetry.addLine(" Area Density Aspect  Center");

            // Display the size (area) and center location for each Blob.
            for(ColorBlobLocatorProcessor.Blob b : blobs)
            {

                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d) %.3f",
                          b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y, calculateScore(b)));
            }

            if (!blobs.isEmpty()){
                ColorBlobLocatorProcessor.Blob BigBlob = blobs.get(0);

                cx = BigBlob.getBoxFit().center.x;
               PIDActive = true;

            }
            else if(targetFound){
                cx = detectedTag.center.x;
                PIDActive = true;
            }
            else{
                cx = 1280/2;
                PIDActive = false;
            }



            double cXerror = (cx-640);

            if (Math.abs(cXerror) > 60) {
                //Rotation = (cX - 640) * 0.0005;
                //LR = controller.calculate(cXerror,0)/2;
            }
            else if (Math.abs(cXerror)<=60 || gamepad1.start){
                //LR = controller.calculate(cXerror,0)/2;
            }

            double mFLPower = FB + LR + Rotation;
            double mFRPower = FB - LR - Rotation;
            double mBLPower = FB - LR + Rotation;
            double mBRPower = FB + LR - Rotation;

            mFL.setPower(mFLPower);
            mFR.setPower(mFRPower);
            mBL.setPower(mBLPower);
            mBR.setPower(mBRPower);


            telemetry.addData("cx", cx);
            telemetry.addData("pdcontroller",controller.calculate(cXerror,0));
            telemetry.addData("LR", LR);
            telemetry.addData("aprilTagDetected", targetFound);

            telemetry.addData("PIDActive?", PIDActive);


            if(targetFound == true){
            telemetry.addData("aprilTagCenter", detectedTag.center.x);
            }

            //telemetry.addData("Detected color", )
            telemetry.update();
            sleep(50);
        }
    }

//    public double getScore(ArrayList a){
//        int d = 0;
//        if (d <= a.size()){
//            double score =
//        }
//        return score;
//
//    }

//    public void sortByScore(List a){
//
//
//    }

}

