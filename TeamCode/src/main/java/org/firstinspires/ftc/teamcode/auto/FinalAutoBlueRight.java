package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.cameradetection.CameraDetectionV2;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name = "FinalAutoBlueRight")
public class FinalAutoBlueRight extends LinearOpMode {

    int parkNumber = 0;
    boolean idFound = false;

    @Override
    public void runOpMode() throws InterruptedException {

        //Calls SampleMecanumDrive to initialize the motors and use the methods.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Creates initial position of the robot based on the back center of the robot
        Pose2d startPose = new Pose2d(-72, 36, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        CameraDetectionV2 camera = new CameraDetectionV2();

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .back(10)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .back(39)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .back(16)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(16)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(16)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .forward(16)
                .build();

        Trajectory parkTraj = null;

        waitForStart();

        if(!isStopRequested()) {
            
            
            if (parkNumber == 1) {
                parkTraj = drive.trajectoryBuilder(new Pose2d())
                        .back(24)
                        .build();
            }
            else if (parkNumber == 2) {
                parkTraj = drive.trajectoryBuilder(new Pose2d())
                        .build();
            }
            else if (parkNumber == 3) {
                parkTraj = drive.trajectoryBuilder(new Pose2d())
                        .forward(24)
                        .build();
            }
            else {
                telemetry.addLine("well frick");
                telemetry.update();
                parkTraj = drive.trajectoryBuilder(new Pose2d())
                        .build();
            }
            drive.followTrajectory(parkTraj);
        }
    }
}
