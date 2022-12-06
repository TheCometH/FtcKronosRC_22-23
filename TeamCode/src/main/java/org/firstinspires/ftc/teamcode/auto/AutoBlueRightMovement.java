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

@Autonomous(name = "AutoBlueRightMovement")
public class AutoBlueRightMovement extends LinearOpMode {



    @Override
    public void runOpMode() {

        //Creates a CameraDetection object to detect the id and return a value for parking
        /*CameraDetection camera = new CameraDetection();
        camera.init();
        camera.detect();
        camera.update();

        //Initialize park to make it equal
        int park = camera.check();*/

        //Calls SampleMecanumDrive to initialize the motors and use the methods.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Creates initial position of the robot based on the back center of the robot
        Pose2d startPose = new Pose2d(-72, -36, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-21, -36))
                //.waitSeconds(2)
                //.turn(Math.toRadians(90))
                .addSpatialMarker(new Vector2d(-62, -36), () -> {
                    telemetry.addLine("Reach (-62, -36)");
                    telemetry.update();
                })
                .addDisplacementMarker(10, () -> {
                    telemetry.addLine("10 inches past");
                    telemetry.update();
                })

                .addSpatialMarker(new Vector2d(-52, -36), () -> {
                    telemetry.addLine("Reach (-52, -36)");
                    telemetry.update();
                })
                .addDisplacementMarker(20, () -> {
                    telemetry.addLine("20 inches past");
                    telemetry.update();
                })

                .addSpatialMarker(new Vector2d(-42, -36), () -> {
                    telemetry.addLine("Reach (-42, -36)");
                    telemetry.update();
                })
                .addDisplacementMarker(30, () -> {
                    telemetry.addLine("30 inches past");
                    telemetry.update();
                })

                .addSpatialMarker(new Vector2d(-32, -36), () -> {
                    telemetry.addLine("Reach (-32, -36)");
                    telemetry.update();
                })
                .addDisplacementMarker(40, () -> {
                    telemetry.addLine("40 inches past");
                    telemetry.update();
                })

                .addSpatialMarker(new Vector2d(-21, -36), () -> {
                    telemetry.addLine("Reach (-21, -36)");
                    telemetry.update();
                })
                .addDisplacementMarker(51, () -> {
                    telemetry.addLine("51 inches past");
                    telemetry.update();
                })
                /*.lineToLinearHeading(new Pose2d(-21,-50, Math.toRadians(90)))
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-21,-36, Math.toRadians(180)))
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-21,-50, Math.toRadians(-180)))
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-21,-36, Math.toRadians(180)))*/
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

        if(!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}