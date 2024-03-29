package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.cameradetection.CameraDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoBlueRight_v2")
public class AutoBlueRight_v2 extends LinearOpMode {
    @Override
    public void runOpMode() {

        //Creates a CameraDetection object to detect the id and return a value for parking
        /*CameraDetection camera = new CameraDetection();

        camera.init();
        camera.detect();
        camera.update();

        //Initialize park to receive parking location
        int park = camera.check();*/
        
        int park = 0;
        //Calls SampleMecanumDrive to initialize the motors and use the methods.
        // CREATES A SAMPLE MECANUM DRIVE
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Creates initial position of the robot based on the back center of the robot
        
        Pose2d startPose = new Pose2d(-72, -36, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        //Approaches the signal cone for the camera to detect the id
        Trajectory cameraTraj = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-20, -36))
                .build();

        //Plows the signal cone and moves to (-21, -36) to allow the robot to drop the cone on the high junction
        Trajectory traj1 = drive.trajectoryBuilder(cameraTraj.end())
                .forward(10)
                .build();

        //Rotates 90 degrees counterclockwise and moves to (-21, -50) to pick up a cone
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(14)
                .build();

        //Rotates 180 degrees counterclockwise and moves to (-21, -36) to drop the cone on the high junction
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(-21,-36, Math.toRadians(180)))
                .build();

        //Rotates 180 degrees clockwise and moves to (-21, -50) to pick up a cone
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(-21,-50, Math.toRadians(-180)))
                .build();

        //Initialize parking trajectory
        Trajectory parkTraj = null;

        if (park == 1) {
            parkTraj = drive.trajectoryBuilder(traj3.end())
                    .forward(24)
                    .build();
        }
        else if (park == 3) {
            parkTraj = drive.trajectoryBuilder(traj3.end())
                    .back(24)
                    .build();
        }
        else {
            telemetry.addLine("well frick");
            telemetry.update();
        }

        waitForStart();

        if(isStopRequested()) return;

        //drive.rotate();
        drive.followTrajectory(cameraTraj);
        telemetry.addLine("ITS GOING FORWARD WOWO");
        telemetry.update();
        sleep(3000);



        //Goes to -21, -36, turni  ng 90 degrees clockwise
       /*
        drive.followTrajectory(traj1);
        telemetry.addLine("WOW IS IT GOING FORWARD WAIT ITS NOT");
        telemetry.update();

        sleep(3000);
        */
        //Code to drop the cone on the top junction
        //drive.rotate(0.1, 1);
        //drive.traversing(0.1, 1);
        //drive.expand(0.1, 1);
        //drive.tiltNow(90);
        //drive.openClaw();

        //Brings the
        //drive.tiltNow();
        //drive.traversing(-0.1, 1);
        //drive.expand(-0.05, 1);

//        drive.followTrajectory(traj2);
  //      sleep(3000);
      //  drive.turn(Math.toRadians(90));
    //    sleep(3000);
        //rotate arm -90 degrees during
        //drive.expand(0.1, 1);
        //drive.rotate(0.1, -0.25);
        //drive.closeClaw();
        //drive.rotate(0.1, 0.25);
        //drive.expand(0.1, 1);
        //drive.tiltNow();
        //drop cone on short junction
        //rotate arm -135 degrees
        //grab cone

//        drive.followTrajectory(traj3);
        //drive.traversing(0.1, 1);
        //drive.expand(0.1, 1);
        //drive.tiltNow(90);
        //drive.openClaw();
        //drive.tiltNow();
        //drive.traversing(0.1, -1);
        //drive.expand(0.1, -1);
        //drop on tall junction

        //park``
//        drive.followTrajectory(traj4);
        //drive.expand(0.1, 1);
        //drive.rotate(0.1, -0.25);
        //drive.closeClaw();
        //drive.rotate(0.1, 0.25);
        //drive.expand(0.1, 1);
        //drive.tiltNow();

//        if (parkTraj != null) {
  //          drive.followTrajectory(parkTraj);
    //    }
    }
}
