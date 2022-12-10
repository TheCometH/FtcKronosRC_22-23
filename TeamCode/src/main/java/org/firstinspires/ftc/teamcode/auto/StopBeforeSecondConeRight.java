package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.auto.cameradetection.CameraDetectionV2;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name = "StopBeforeSecondConeRight")
public class StopBeforeSecondConeRight extends LinearOpMode {

    int parkNumber = 0;
    boolean idFound = true;

    DcMotorEx expansion;
    DcMotorEx rotation;
    DcMotorEx traverse;
    Servo tilt;
    Servo clamp;

    private static double arm_ticks_per_rev = 5281.1 * 2;
    public static double expand_ticks_per_rev = 384.5;
    //4.319 revolutions, 1650 max
    public static double armticks_per_angle= arm_ticks_per_rev/360;
    static int ticks;

    HardwareMap hwmap = null;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        //Calls SampleMecanumDrive to initialize the motors and use the methods.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Creates initial position of the robot based on the back center of the robot
        Pose2d startPose = new Pose2d(-72, 36, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        CameraDetectionV2 camera = new CameraDetectionV2();

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .back(14)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .back(35.5)
                .build();

        Trajectory parkTraj = null;

        camera.initTele(telemetry);
        camera.initCamera(hardwareMap);

        waitForStart();

        if(!isStopRequested()) {

            rotate(1, 60);
            traversing(-0.5, -5);
            tilt.setPosition(0.75);
            drive.followTrajectory(traj0);

            /*
             * The INIT-loop:
             * This REPLACES waitForStart!
             */
            int xp = 0;
            while (isStarted() && !isStopRequested() && idFound)
            {
                try {
                    camera.detect();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                telemetry.addData("Inside looop", xp);
                telemetry.update();
                xp++;
                /*
                 * The START command just came in: now work off the latest snapshot acquired
                 * during the init loop.
                 */

                /* Update the telemetry */
                camera.update();

                /* Actually do something useful */
                parkNumber = camera.check();

                if (parkNumber == 1 || parkNumber == 2 || parkNumber == 3) {
                    idFound = false;
                    break;
                }
                else {
                    telemetry.addLine("not found");
                    telemetry.update();
                }
            }
            telemetry.addLine("parking spot: " + parkNumber);
            telemetry.update();
            sleep(8000);

            if (parkNumber == 1) {
                parkTraj = drive.trajectoryBuilder(traj1.end())
                        .strafeRight(24)
                        .build();
            }
            else if (parkNumber == 3) {
                parkTraj = drive.trajectoryBuilder(traj1.end())
                        .strafeLeft(24)
                        .build();
            }
            else {
                telemetry.addLine("well frick");
                telemetry.update();
                parkTraj = drive.trajectoryBuilder(traj1.end())
                        .forward(1)
                        .build();
            }

            Trajectory traj2 = drive.trajectoryBuilder(parkTraj.end())
                    .forward(12)
                    .build();

            drive.followTrajectory(traj1);
            rotate(1, 63);
            traversing(-0.7, -53);
            expand(1, 950);
            tilt.setPosition(0.2);
            sleep(500);

            openClaw();
            sleep(500);
            expand(-0.4, -940);
            traversing(0.7, 61);
            rotate(1, 27);
            //rotate(-1, -63);


//            drive.followTrajectory(traj2);
//            rotate(-1, -10);
//            tilt.setPosition(0.55);
//            sleep(500);
//            expand(1, 1144);
//            closeClaw();
//            sleep(500);
//            rotate(1, 10);
//            expand(-0.4, -1144);
//
//            drive.followTrajectory(traj3);
//            traversing(-0.7, -70);
//            rotate(1, 63);
//            traversing(-0.7, -68);
//            expand(1, 1077);
//            tilt.setPosition(0.2);
//            sleep(500);
//            openClaw();
//            sleep(500);
//            closeClaw();
//            sleep(500);
//            expand(-0.5, -1077);
//            traversing(0.7, 68);
//            rotate(-1, -63);

            drive.followTrajectory(parkTraj);
            drive.followTrajectory(traj2);
            rotate(-1, -90);
        }
    }

    public void initialize() {
        rotation = hardwareMap.get(DcMotorEx.class, "rotation");
        expansion = hardwareMap.get(DcMotorEx.class, "expansion");
        traverse = hardwareMap.get(DcMotorEx.class, "traverse");
        tilt = hardwareMap.get(Servo.class, "tilt");
        clamp = hardwareMap.get(Servo.class, "clamp");

        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        expansion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        traverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotation.setDirection(DcMotorSimple.Direction.REVERSE);
        expansion.setDirection(DcMotorSimple.Direction.FORWARD);
        traverse.setDirection(DcMotorSimple.Direction.FORWARD);
        tilt.setDirection(Servo.Direction.FORWARD);
        clamp.setDirection(Servo.Direction.FORWARD);

        clamp.setPosition(0);
        rotation.setPower(0);
        expansion.setPower(0);
        traverse.setPower(0);
    }

    public void rotate(double power, int angle) {
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int distance = (int) (armticks_per_angle * angle);

        rotation.setTargetPosition(distance);

        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rotation.setPower(power);

        while (rotation.isBusy()) {
            telemetry.addData("In rotation while loop", rotation.getCurrentPosition());
            telemetry.update();
        }

        rotation.setPower(0);
        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void expand(double power, int distance) {
        expansion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        expansion.setTargetPosition(distance);

        expansion.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        expansion.setPower(power);

        while (expansion.isBusy()) {
            telemetry.addData("In expansion while loop", expansion.getCurrentPosition());
            telemetry.update();

            if (Math.abs(expansion.getCurrentPosition() - distance) < 200){
                break;
            }

        }

        expansion.setPower(0);
        expansion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void traversing(double power, int angle) {
        traverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int distance = (int) (armticks_per_angle * angle);

        traverse.setTargetPosition(distance);

        traverse.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        traverse.setPower(power);
        while (traverse.isBusy()) {
            telemetry.addData("In traverse while loop", traverse.getCurrentPosition());

            telemetry.update();
        }

        traverse.setPower(0);
        traverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int distance(double angle) {
        ticks = (int) (angle * armticks_per_angle);
        return ticks;
    }

    public void closeClaw() {
        clamp.setPosition(0);
    }

    public void openClaw() {
        clamp.setPosition(0.5);
    }
}
