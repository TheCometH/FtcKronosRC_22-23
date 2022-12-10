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

@Autonomous(name = "AutoBlueRightMovement")
public class AutoBlueRightMovement extends LinearOpMode {

    int parkNumber = 0;
    boolean idFound = false;

    @Override
    public void runOpMode() throws InterruptedException {

        //Creates a CameraDetection object to detect the id and return a value for parking

        //Calls SampleMecanumDrive to initialize the motors and use the methods.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Creates initial position of the robot based on the back center of the robot
        Pose2d startPose = new Pose2d(-72, -36, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        CameraDetectionV2 camera = new CameraDetectionV2();
        Trajectory parkTraj;

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(10)
                .waitSeconds(5)
                .addTemporalMarker(0,() -> {
                    camera.initTele(telemetry);
                    camera.initCamera(hardwareMap);

                    /*
                     * The INIT-loop:
                     * This REPLACES waitForStart!
                     */
                    while (!isStarted() && !isStopRequested() && !idFound)
                    {
                        try {
                            camera.detect();
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        /*
                         * The START command just came in: now work off the latest snapshot acquired
                         * during the init loop.
                         */

                        /* Update the telemetry */
                        camera.update();

                        /* Actually do something useful */
                        parkNumber = camera.check();

                        if (parkNumber == 1 || parkNumber == 2 || parkNumber == 3) {
                            idFound = true;
                        }
                        else {
                            telemetry.addLine("well frick");
                            telemetry.update();
                        }
                    }
                    telemetry.addLine("parking spot: " + parkNumber);
                    telemetry.update();

                })
                .back(39)
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .back(16)
                .waitSeconds(1)
                .forward(16)
                .waitSeconds(1)
                .turn(Math.toRadians(180))
                .build();

        /*TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-60,-36),
                        SampleMecanumDrive.getVelocityConstraint(3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                //Goes 3 in/s towards the position
                .waitSeconds(3)
                .lineTo(new Vector2d(-21, -36))
                .waitSeconds(1.5)
                .turn(Math.toRadians(90))
                .waitSeconds(1.5)
                .lineTo(new Vector2d(-21, -50))
                .waitSeconds(1.5)
                .turn(Math.toRadians(180))
                .waitSeconds(1.5)
                .lineTo(new Vector2d(-21, -36))
                .waitSeconds(1.5)
                .lineTo(new Vector2d(-21, -50))
                .waitSeconds(1.5)
                .turn(Math.toRadians(180))
                .waitSeconds(1.5)

                .lineToLinearHeading(new Pose2d(-21,-50, Math.toRadians(-180)))
                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(-21,-36, Math.toRadians(180)))
                .build();
*/
        waitForStart();

        if(!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
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