package org.firstinspires.ftc.teamcode.Detection;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class PropDetectionRedFar implements VisionProcessor {

    public int detection = 1;

    public static int rightRectX1 = 350, rightRectY1 = 350;
    public static int rightRectX2 = 450, rightRectY2 = 450;

    public static double rightThresh = 400000;
    public double rightSum = 0;

    public static int middleRectX1 = 670, middleRectY1 = 350;
    public static int middleRectX2 = 770, middleRectY2 = 450;

    public static double middleThresh = 500000;
    public double middleSum = 0;

    public static int redLowH = 110, redLowS = 160, redLowV = 0;
    public static int redHighH = 125, redHighS = 255, redHighV = 255;

    Mat workingMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, workingMat, Imgproc.COLOR_BGR2HSV);

        Rect rightRect = new Rect(new Point(rightRectX1, rightRectY1), new Point(rightRectX2, rightRectY2));
        Rect middleRect = new Rect(new Point(middleRectX1, middleRectY1), new Point(middleRectX2, middleRectY2));

        Scalar lowThresh = new Scalar(redLowH, redLowS, redLowV);
        Scalar highThresh = new Scalar(redHighH, redHighS, redHighV);

        Core.inRange(workingMat, lowThresh, highThresh, workingMat);

        rightSum = Core.sumElems(workingMat.submat(rightRect)).val[0];
        middleSum = Core.sumElems(workingMat.submat(middleRect)).val[0];

        Imgproc.rectangle(frame, rightRect, new Scalar(0,255,0), 5);
        Imgproc.rectangle(frame, middleRect, new Scalar(0,255,0), 5);

        if(rightSum > rightThresh)
            detection = 1;
        else if (middleSum > middleThresh)
            detection = 2;
        else detection = 3;

//        workingMat.copyTo(frame);

        workingMat.release();

        return detection;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}