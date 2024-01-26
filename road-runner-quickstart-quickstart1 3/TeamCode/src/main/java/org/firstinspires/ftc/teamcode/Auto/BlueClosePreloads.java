package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Detection.PropDetectionBlueClose;
import org.firstinspires.ftc.teamcode.Detection.PropDetectionRedClose;
import org.firstinspires.ftc.teamcode.Detection.TeamPropPipelineRed;
import org.firstinspires.ftc.teamcode.Hardware.PIDConstants;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Detection.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name ="🟦BlueClose", group = "CENTERSTAGE")

public class BlueClosePreloads extends LinearOpMode {

    SampleMecanumDrive drive;
    TeleOp teleop = new TeleOp();
    Pose2d startPose = new Pose2d(11, 64.5, Math.toRadians(270));

    Trajectory pos1;
    Trajectory pos2;

    Trajectory pos3;

    Trajectory pos4;
    Trajectory pos5;

    Trajectory pos6;
    Trajectory pos7;
    Trajectory pos8;
    Trajectory pos9;
    Trajectory posinspate;
    Trajectory posint;
    Trajectory posplace;
    Trajectory postras;
    Trajectory pospixeli;

    CASE Case;

    Servo cycle;
    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    double cX = 0;
    double cY = 0;
    double width = 0;




    enum Location {
        Left,
        Center,
        Right
    }


    public enum CASE {
        left,
        center,
        right
    }

    int cazzz = 2;
    PropPipeline PropPipeline = new PropPipeline();


    public static double targetPosition = 0;
    TeamPropPipelineRed teamPropPieline = new TeamPropPipelineRed();
    VisionPortal portal;
    PropDetectionBlueClose processor;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        teleop.lift.initLiftAuto(hardwareMap);
        teleop.intake.intakeinit(hardwareMap);
        drive.setPoseEstimate(startPose);
        CASE cazul = CASE.right;
        cycle = hardwareMap.get(Servo.class, "cycle");
        cycle.setPosition(0.15);
        processor = new PropDetectionBlueClose();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(1280,720))
                .enableLiveView(true)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();

        while (opModeInInit()) {

            telemetry.update();
            telemetry.addData("Detection", processor.detection);
            telemetry.addData("Sum left", processor.rightSum);
            telemetry.addData("Middle sum", processor.middleSum);

            telemetry.update();
            telemetry.update();

        }


        if(opModeIsActive() && !isStopRequested()) {
            if(processor.detection==3){
                right();
            }

            if(processor.detection==2){
                center();
            }
            if(processor.detection==1){
                left();
            }
        }


    }


    public void center() {
        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(10, 35, Math.toRadians(260)), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(12, 48, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(49.6, 34.8, Math.toRadians(1)), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos4 = drive.trajectoryBuilder(pos3.end())
                .lineToLinearHeading(new Pose2d(41, 15, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        drive.followTrajectory(pos1);
        sleep(100);
        teleop.lift.preload();
        drive.followTrajectory(pos2);
        teleop.lift.preloadServo();
        drive.followTrajectory(pos3);
        sleep(400);
        teleop.lift.servoPixel.setPower(0.7);
        sleep(2000);
        drive.followTrajectory(pos4);
        teleop.lift.RetractServo();
        sleep(500);
        teleop.lift.Retract();
        teleop.lift.servoPixel.setPower(0);
        sleep(3000);


    }

    public void right() {
        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(2, 35, Math.toRadians(250)), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(8, 51, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(47.8, 40, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos4 = drive.trajectoryBuilder(pos3.end())
                .lineToLinearHeading(new Pose2d(40, 15, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        drive.followTrajectory(pos1);
        sleep(100);
        teleop.lift.preload();
        drive.followTrajectory(pos2);
        teleop.lift.preloadServo();
        drive.followTrajectory(pos3);
        sleep(400);
        teleop.lift.servoPixel.setPower(0.7);
        sleep(2000);
        drive.followTrajectory(pos4);
        teleop.lift.RetractServo();
        sleep(500);
        teleop.lift.Retract();
        teleop.lift.servoPixel.setPower(0);
        sleep(1500);


    }

    public void left() {
        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(14, 37.5, Math.toRadians(300)), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(12, 45, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(48, 42, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos4 = drive.trajectoryBuilder(pos3.end())
                .lineToLinearHeading(new Pose2d(40, 15, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        drive.followTrajectory(pos1);
        sleep(100);
        teleop.lift.preload();
        drive.followTrajectory(pos2);
        teleop.lift.preloadServo();
        drive.followTrajectory(pos3);
        sleep(400);
        teleop.lift.servoPixel.setPower(0.7);
        sleep(2000);
        drive.followTrajectory(pos4);
        teleop.lift.RetractServo();
        sleep(500);
        teleop.lift.Retract();
        teleop.lift.servoPixel.setPower(0);
        sleep(1500);


    }


    public void collect() {
        teleop.intake.intakeMotor.setPower(-1);
        teleop.intake.intakeMotorRight.setPower(1);
        teleop.lift.servoPixel.setPower(-1);
    }

    public void exit() {
        teleop.intake.intakeMotor.setPower(-1);
        teleop.intake.intakeMotorRight.setPower(1);
        teleop.lift.servoPixel.setPower(-1);
    }

    public void stopintake() {
        teleop.intake.intakeMotor.setPower(0);
        teleop.intake.intakeMotorRight.setPower(0);
        teleop.lift.servoPixel.setPower(0);
    }





}
