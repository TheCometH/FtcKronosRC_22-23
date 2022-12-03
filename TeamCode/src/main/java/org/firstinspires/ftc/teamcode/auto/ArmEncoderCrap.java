package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class ArmEncoderCrap extends LinearOpMode {

    DcMotorEx expansion;
    DcMotorEx rotation;
    DcMotorEx traverse;
    Servo tilt;
    Servo clamp;

    @Override
    public void runOpMode() throws InterruptedException {
        // trajectory to tall pole (theoretically)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(12, 46.6666666666))
                .splineTo(new Vector2d(12, 72), 0)
                .addDisplacementMarker( () -> {
                    // estimations on 90 deg to roate it to the right height
                    drive.rotate(0.5, 360);
                    // change the distance to however much the expansion needs
                    drive.expand(0.1, 2880);

                    drive.stopArmMovement();
                })
                .splineTo(new Vector2d(48, 72), Math.toRadians(90))
                .build();


        waitForStart();
        //if(isStopRequested()) return;
        //drive.followTrajectory(traj1);
    }
}