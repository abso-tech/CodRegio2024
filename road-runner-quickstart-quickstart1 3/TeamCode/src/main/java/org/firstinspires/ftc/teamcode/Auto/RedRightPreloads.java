package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Detection.PropDetectionRedClose;
import org.firstinspires.ftc.teamcode.Detection.TeamPropPipelineRed;
import org.firstinspires.ftc.teamcode.Hardware.PIDConstants;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Detection.PropPipeline;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
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

@Autonomous(name ="🛑RedClosePreloads", group = "CENTERSTAGE")

public class RedRightPreloads extends LinearOpMode {

    SampleMecanumDrive drive;
    TeleOp teleop = new TeleOp();
    Pose2d startPose = new Pose2d(11, -64.5, Math.toRadians(90));

    Trajectory pos1;
    Trajectory pos2;

    TrajectorySequence pos3;

    Trajectory pos4;
    Trajectory pos5;

    Trajectory pos6;
    Trajectory pos7;
    Trajectory pos8;
    Trajectory pos9;

    Trajectory pos10;
    Trajectory posinspate;
    Trajectory posint;
    Trajectory posplace;
    Trajectory postras;
    Trajectory pospixeli;

    CASE Case;

    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    /** MAKE SURE TO CHANGE THE FOV AND THE RESOLUTIONS ACCORDINGLY **/
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
    private static final double FOV = 40;

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels


    enum Location{
        Left,
        Center,
        Right
    }




    public enum CASE {
        left,
        center,
        right
    }

    int cazzz=2;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
   PropPipeline PropPipeline = new PropPipeline();


    public static double targetPosition = 0;
    TeamPropPipelineRed teamPropPieline = new TeamPropPipelineRed();
    VisionPortal portal;
    PropDetectionRedClose processor;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        teleop.lift.initLiftAuto(hardwareMap);
        teleop.intake.intakeinit(hardwareMap);
        teleop.airplane.initAirplane(hardwareMap);
        drive.setPoseEstimate(startPose);
        CASE cazul = CASE.right;
        processor = new PropDetectionRedClose();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(1280, 720))
                .enableLiveView(true)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();


        int detectionCase = 1;
//mov
        pos1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(11, -35, Math.toRadians(90)),Math.toRadians(90),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
//mov fata
        pos2 = drive.trajectoryBuilder(pos1.end())
                .splineToLinearHeading(new Pose2d(11, -44, Math.toRadians(90)),Math.toRadians(90),SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

//toGalben
        pos3 = drive.trajectorySequenceBuilder(pos2.end())
                .splineTo(new Vector2d(30, -44),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(45, -41),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1,()->{
                    teleop.lift.preload();
                })
                .addTemporalMarker(2,()->{
                    teleop.lift.preloadServo();
                })
                .build();
//leaveGalben
        pos5 = drive.trajectoryBuilder(pos3.end(),true)
                .splineToConstantHeading(new Vector2d(20, -59),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.6,()->{
                    teleop.lift.RetractServo();

                })
                .addTemporalMarker(1.1,()->{
                    teleop.lift.Retract();
                    teleop.lift.servoPixel.setPower(0);

                })
                .build();
//toStack
        pos6 = drive.trajectoryBuilder(pos5.end(),true)
                .splineToConstantHeading(new Vector2d(0, -59),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-16, -58),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-32, -57),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-40, -56),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-65, -31.5),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1,()->{
                    teleop.intake.intakeUAD.setPosition(0.25);
                    collect();
                })
                .build();
//toPannel
        pos7 = drive.trajectoryBuilder(pos6.end())
                .splineTo(new Vector2d(-55, -48),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-32, -47),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(new Vector2d(-16, -46.5),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(new Vector2d(0, -46),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(new Vector2d(16, -45),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(27, -43),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3,()->{
                    exit();
                })

                .addTemporalMarker(2.7,()->{
                    teleop.lift.cycle();
                })
                .addTemporalMarker(3.3,()->{
                    teleop.lift.preloadServo();
                })
                .splineTo(new Vector2d(45, -38),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


//toStack  t2
        pos8 = drive.trajectoryBuilder(pos7.end(),true)
                .splineToConstantHeading(new Vector2d(20, -48),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.4,()->{
                    teleop.lift.RetractServo();

                })
                .addTemporalMarker(0.8,()->{
                    teleop.lift.Retract();
                    teleop.lift.servoPixel.setPower(0);

                })
                .build();


        pos9 = drive.trajectoryBuilder(pos8.end(),true)

//                .splineTo(new Vector2d(0, -46),Math.toRadians(190),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-16, -47),Math.toRadians(190),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-32, -45),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-42, -38),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .addTemporalMarker(0.1,()->{
                    teleop.intake.intakeUAD.setPosition(0.34);
                    collect();
                })
                .splineTo(new Vector2d(-65, -15),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();




//to pannel t2
        pos10 = drive.trajectoryBuilder(pos9.end())
                .splineTo(new Vector2d(-55, -39),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(new Vector2d(-32, -39),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(new Vector2d(-16, -39),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(new Vector2d(0, -39),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(new Vector2d(22, -39),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-32, -38),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(27, -37.5),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3,()->{
                    exit();
                })
                .addTemporalMarker(2.7,()->{
                    teleop.lift.cycle();
                })
                .addTemporalMarker(3.3,()->{
                    teleop.lift.preloadServo();
                })
                .splineTo(new Vector2d(35, -35),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();









        while (opModeInInit()) {
            dashboard.setTelemetryTransmissionInterval(55);
            detectionCase = processor.detection;
            telemetry.addData("Detection", processor.detection);
            telemetry.update();

        }

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            center();
        }
    }


    public void center(){




        drive.followTrajectory(pos1);
        drive.followTrajectory(pos2);
        drive.setPoseEstimate(pos2.end());
        drive.followTrajectorySequence(pos3);
        teleop.lift.servoPixel.setPower(1);
        sleep(250);
        drive.setPoseEstimate(pos3.end());
        drive.followTrajectory(pos5);
        drive.followTrajectory(pos6);
        teleop.intake.intakeUAD.setPosition(0.3);
        sleep(50);
        teleop.intake.intakeUAD.setPosition(0.32);
        collect();
        sleep(50);
        drive.setPoseEstimate(pos6.end());
        drive.followTrajectory(pos7);
        teleop.lift.servoPixel.setPower(1);
        sleep(300);
        drive.setPoseEstimate(pos7.end());
        drive.followTrajectory(pos8);
        drive.followTrajectory(pos9);

//        sleep(200);
        drive.setPoseEstimate(pos9.end());
        drive.followTrajectory(pos10);
        teleop.lift.servoPixel.setPower(1);
        sleep(400);
        teleop.lift.RetractServo();
        sleep(400);
        teleop.lift.Retract();
        teleop.lift.servoPixel.setPower(0);








//        teleop.intake.intakeUAD.setPosition(0.45);
//        collect();


//        drive.followTrajectorySequence(pos5);

//        drive.followTrajectory(pos4);




    }

//    public void right(){
//        pos1 = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(11, -35, Math.toRadians(45)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//        pos2 = drive.trajectoryBuilder(pos1.end())
//                .lineToLinearHeading(new Pose2d(8, -50, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//
//        pos3= drive.trajectorySequenceBuilder(pos2.end())
//                .lineToLinearHeading(new Pose2d(47.8, -47, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
////        pos5= drive.trajectoryBuilder(pos3.end())
////                .back(4)
////                .build();
//        pos4= drive.trajectoryBuilder(pos5.end())
//                .lineToLinearHeading(new Pose2d(39, -62, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//
//
//
//        drive.followTrajectory(pos1);
//        sleep(100);
//        teleop.lift.preload();
//        drive.followTrajectory(pos2);
//        teleop.lift.preloadServo();
//        drive.followTrajectory(pos3);
//        sleep(400);
//        teleop.lift.servoPixel.setPower(0.7);
//        sleep(2000);
////        drive.followTrajectory(pos5);
//        drive.followTrajectory(pos4);
//        teleop.lift.RetractServo();
//        sleep(500);
//        teleop.lift.Retract();
//        teleop.lift.servoPixel.setPower(0);
//        sleep(25000);
//
//
//
//    }
//    public void left(){
//        pos1 = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(1, -37.5, Math.toRadians(134)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//        pos2 = drive.trajectoryBuilder(pos1.end())
//                .lineToLinearHeading(new Pose2d(12, -45, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//
//        pos3= drive.trajectoryBuilder(pos2.end())
//                .lineToLinearHeading(new Pose2d(46.8, -33.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
////        pos5= drive.trajectoryBuilder(pos3.end())
////                .back(6)
////                .build();
////
//
//        pos4= drive.trajectoryBuilder(pos5.end())
//                .lineToLinearHeading(new Pose2d(39, -62, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//
//
//
//        drive.followTrajectory(pos1);
//        sleep(100);
//        teleop.lift.preload();
//        drive.followTrajectory(pos2);
//        teleop.lift.preloadServo();
//        drive.followTrajectory(pos3);
//        sleep(400);
//        teleop.lift.servoPixel.setPower(0.7);
//        sleep(2000);
////        drive.followTrajectory(pos5);
//        drive.followTrajectory(pos4);
//        teleop.lift.RetractServo();
//        sleep(500);
//        teleop.lift.Retract();
//        teleop.lift.servoPixel.setPower(0);
//        sleep(25000);
//
//
//
//    }


    public void collect (){
        teleop.intake.intakeMotor.setPower(-1);
        teleop.intake.intakeMotorRight.setPower(-1);
        teleop.lift.servoPixel.setPower(-1);
    }
    public void exit (){
        teleop.intake.intakeMotor.setPower(1);
        teleop.intake.intakeMotorRight.setPower(1);
        teleop.lift.servoPixel.setPower(0);
    }

    public void stopintake (){
        teleop.intake.intakeMotor.setPower(0);
        teleop.intake.intakeMotorRight.setPower(0);
        teleop.lift.servoPixel.setPower(0);
    }







}
