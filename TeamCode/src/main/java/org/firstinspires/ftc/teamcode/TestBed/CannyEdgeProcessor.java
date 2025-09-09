package org.firstinspires.ftc.teamcode.TestBed;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.TestBed.SDKVisionColorLocatorTYFIRST.imgHeight;
import static org.firstinspires.ftc.teamcode.TestBed.SDKVisionColorLocatorTYFIRST.imgWidth;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.vision.VisionProcessor;

public  class CannyEdgeProcessor implements VisionProcessor {
    private Mat gray = new Mat();
    private Mat edges = new Mat();


    public void init(int width, int height, CameraCalibration calibration) {
        // Nothing special to initialize here

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert to grayscale
        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGBA2GRAY);

        // Apply Canny Edge Detection
        Imgproc.Canny(gray, edges, 100, 200);

        // Draw edges back onto frame for visualization
        Imgproc.cvtColor(edges, frame, Imgproc.COLOR_GRAY2RGBA);

        return null; // No detections to return, just modifies the preview
    }

    @Override
    public void onDrawFrame(android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Optional: could overlay debug info here
    }

}

