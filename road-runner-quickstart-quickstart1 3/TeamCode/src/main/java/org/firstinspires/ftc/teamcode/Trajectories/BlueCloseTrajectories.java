package org.firstinspires.ftc.teamcode.Trajectories;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
public class BlueCloseTrajectories {

    SampleMecanumDrive drive;
    Trajectory purplePiexel,backPurple,BackDropYellow,Parking;

    //poses
        Pose2d startPoseBlue = new Pose2d(11, 64.5, Math.toRadians(270));
        Pose2d PurpleCenter = new Pose2d(10, 35, Math.toRadians(260));
        Pose2d backPurpleCenter =  new Pose2d(12, 48, Math.toRadians(0));
        Pose2d BackDropCenter = new Pose2d(49.6, 34.8, Math.toRadians(1));
        Pose2d FarParking = new Pose2d(41, 15, Math.toRadians(0));
        Pose2d PurpleLeft = new Pose2d(14, 37.5, Math.toRadians(300));
        Pose2d backPurpleLeft = new Pose2d(12, 45, Math.toRadians(0));
        Pose2d BackDropLeft = new Pose2d(48, 42, Math.toRadians(0));


        Pose2d PurpleRight = new Pose2d(2, 35, Math.toRadians(250));
        Pose2d backPurpleRight = new Pose2d(8, 51, Math.toRadians(0));
        Pose2d BackDropRight =new Pose2d(47.8, 40, Math.toRadians(0));



    public void Center(){
        purplePiexel = drive.trajectoryBuilder(startPoseBlue)
                .lineToLinearHeading(PurpleCenter, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        backPurple = drive.trajectoryBuilder(purplePiexel.end())
                .lineToLinearHeading(backPurpleCenter, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        BackDropYellow = drive.trajectoryBuilder(backPurple.end())
                .lineToLinearHeading(BackDropCenter, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Parking = drive.trajectoryBuilder(BackDropYellow.end())
                .lineToLinearHeading(FarParking, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


    }

    public void Left(){
        purplePiexel = drive.trajectoryBuilder(startPoseBlue)
                .lineToLinearHeading(PurpleLeft, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        backPurple = drive.trajectoryBuilder(purplePiexel.end())
                .lineToLinearHeading(backPurpleLeft, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        BackDropYellow = drive.trajectoryBuilder(backPurple.end())
                .lineToLinearHeading(BackDropLeft, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Parking = drive.trajectoryBuilder(BackDropYellow.end())
                .lineToLinearHeading(FarParking, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


    }

    public void Right(){
        purplePiexel = drive.trajectoryBuilder(startPoseBlue)
                .lineToLinearHeading(PurpleLeft, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        backPurple = drive.trajectoryBuilder(purplePiexel.end())
                .lineToLinearHeading(backPurpleLeft, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        BackDropYellow = drive.trajectoryBuilder(backPurple.end())
                .lineToLinearHeading(BackDropLeft, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Parking = drive.trajectoryBuilder(BackDropYellow.end())
                .lineToLinearHeading(FarParking, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


    }


    public enum Case{
        Left, Center, Right
    }

   public void BetterBlueTrajectoriesInit(SampleMecanumDrive drive,int Detection){
    switch (Detection){
        case 1:
            Left();
            break;
        case 2:
           Center();
            break;
        case 3:
            Right();
            break;
    }

    }

}
