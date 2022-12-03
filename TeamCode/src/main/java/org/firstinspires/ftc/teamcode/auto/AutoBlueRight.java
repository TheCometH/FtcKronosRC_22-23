package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.cameradetection.CameraDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class AutoBlueRight extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Creates a CameraDetection object to detect the id and return a value for parking
        CameraDetection camera = new CameraDetection();
        camera.init();
        camera.detect();
        camera.update();

        //Initialize park to make it equal
        int park = camera.check();


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-72, -36, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-60,-36), Math.toRadians(0))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(-60,-60), Math.toRadians(0))
                .splineTo(new Vector2d(-33,-60), Math.toRadians(0))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(-12,-36), Math.toRadians(90))
                .build();

        Trajectory parkTraj;

        if (park == 1) {
            parkTraj = drive.trajectoryBuilder(traj3.end())
                    .forward(24)
                    .build();
        }
        else if (park == 2) {
            parkTraj = drive.trajectoryBuilder(traj3.end())
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
            parkTraj = drive.trajectoryBuilder(traj3.end())
                    .build();
        }

        drive.followTrajectory(traj1);
        //scan with camera
        drive.followTrajectory(traj2); //roate arm -90 degrees during
        //drop cone on short junction
        //rotate arm -135 degrees
        //grab cone
        drive.followTrajectory(traj3);
        //drop on tall junction
        //park
        drive.followTrajectory(parkTraj);
    }
}
