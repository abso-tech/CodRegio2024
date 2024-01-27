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
public class PropDetectionBlueClose implements VisionProcessor {

    public int detection = 2;

    public static int leftRectX1 = 890, leftRectY1 = 120;
    public static int leftRectX2 = 990, leftRectY2 = 250;

    public static double rightThresh = 500000;
    public double rightSum = 0;

    public static int middleRectX1 = 500, middleRectY1 = 240;
    public static int middleRectX2 = 600, middleRectY2 = 340;


    public static double middleThresh = 180000;
    public double middleSum = 0;

    public static int blueLowH = 100, blueLowS = 40, blueLowV = 0;
    public static int blueHighH = 120, blueHighS = 180, blueHighV = 255;

    Mat workingMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, workingMat, Imgproc.COLOR_RGB2HSV);

        Rect rightRect = new Rect(new Point(leftRectX1, leftRectY1), new Point(leftRectX2, leftRectY2));
        Rect middleRect = new Rect(new Point(middleRectX1, middleRectY1), new Point(middleRectX2, middleRectY2));

        Scalar lowThresh = new Scalar(blueLowH, blueLowS, blueLowV);
        Scalar highThresh = new Scalar(blueHighH, blueHighS, blueHighV);

        Core.inRange(workingMat, lowThresh, highThresh, workingMat);

        rightSum = Core.sumElems(workingMat.submat(rightRect)).val[0];
        middleSum = Core.sumElems(workingMat.submat(middleRect)).val[0];

        Imgproc.rectangle(frame, rightRect, new Scalar(0,255,0), 5);
        Imgproc.rectangle(frame, middleRect, new Scalar(0,255,0), 5);

        if(rightSum < rightThresh&& rightSum<400000 )
            detection = 1;
        else if (middleSum > middleThresh)
            detection = 2;
        else detection = 3;

//        workingMat.copyTo(frame);

        workingMat.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}