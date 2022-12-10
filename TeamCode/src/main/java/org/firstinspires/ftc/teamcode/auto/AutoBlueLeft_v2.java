package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.cameradetection.CameraDetectionV2;

@Autonomous(name = "AutoBlueLeft_v2")
public class AutoBlueLeft_v2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CameraDetectionV2 camera = new CameraDetectionV2();

        //Initialize park to receive parking location
        int parkNumber = 0;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-72, 36, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-60, 36),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .lineTo(new Vector2d(-21, 36))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-21, 54, Math.toRadians(180)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(-21,36), Math.toRadians(180))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(-21,54), Math.toRadians(180))
                .build();

        Trajectory parkTraj;


        //Camera detects
        if (!isStopRequested());

        waitForStart();

        drive.followTrajectory(traj0);

        camera.initTele(telemetry);
        camera.initCamera(hardwareMap);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            camera.detect();
            /*
             * The START command just came in: now work off the latest snapshot acquired
             * during the init loop.
             */

            /* Update the telemetry */
            camera.update();

            /* Actually do something useful */
            parkNumber = camera.check();
        }
        telemetry.addLine("parking spot: " + parkNumber);
        telemetry.update();
        sleep(20);

        if (parkNumber == 1) {
            parkTraj = drive.trajectoryBuilder(traj3.end())
                    .forward(24)
                    .build();
        }
        else if (parkNumber == 2) {
            parkTraj = drive.trajectoryBuilder(traj3.end())
                    .build();
        }
        else if (parkNumber == 3) {
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
        sleep(3000);
        drive.followTrajectory(traj1);
        //drive.rotate(0.1, 1);
        //drive.traversing(0.1, 1);
        //drive.expand(0.1, 1);
        //drive.tiltNow(90);
        //drive.openClaw();
        //drive.tiltNow();
        //drive.traversing(0.1, -1);
        //drive.expand(0.1, -1);

        drive.followTrajectory(traj2); //rotate arm -90 degrees during
        //drive.expand(0.1, 1);
        //drive.rotate(0.1, -0.25);
        //drive.closeClaw();
        //drive.rotate(0.1, 0.25);
        //drive.expand(0.1, 1);
        //drive.tiltNow();
        //drop cone on short junction
        //rotate arm -135 degrees
        //grab cone
        drive.followTrajectory(traj3);
        //drive.traversing(0.1, 1);
        //drive.expand(0.1, 1);
        //drive.tiltNow(90);
        //drive.openClaw();
        //drive.tiltNow();
        //drive.traversing(0.1, -1);
        //drive.expand(0.1, -1);
        //drop on tall junction

        //park
        drive.followTrajectory(traj4);
        //drive.expand(0.1, 1);
        //drive.rotate(0.1, -0.25);
        //drive.closeClaw();
        //drive.rotate(0.1, 0.25);
        //drive.expand(0.1, 1);
        //drive.tiltNow();

        drive.followTrajectory(parkTraj);
    }
}
