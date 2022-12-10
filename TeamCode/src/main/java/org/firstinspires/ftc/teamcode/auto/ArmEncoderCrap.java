package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.hardware.HardwareMap;
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

@Config
@Autonomous
public class ArmEncoderCrap extends LinearOpMode {

    DcMotorEx expansion;
    DcMotorEx rotation;
    DcMotorEx traverse;
    Servo tilt;
    Servo clamp;

    private static double arm_ticks_per_rev = 5281.1 * 2;
    public static double expand_ticks_per_rev = 384.5;
    public static double ticks_per_angle = arm_ticks_per_rev/360;
    static int ticks;

    HardwareMap hwmap = null;

    @Override
    public void runOpMode() throws InterruptedException {

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

        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        traverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        expansion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        tilt.setPosition(0.705);
        clamp.setPosition(0);
        rotation.setPower(0);
        expansion.setPower(0);
        traverse.setPower(0);

        waitForStart();

        int rotationInitialPosition = rotation.getCurrentPosition();
        telemetry.addData("Rotation Initial Position: ", rotationInitialPosition);
        rotate(1, (int) (arm_ticks_per_rev/4));

        int rotationHalfRevolutionPosition = rotation.getCurrentPosition();
        telemetry.addData("Rotation Half Revolution Position: ", rotationHalfRevolutionPosition);
        telemetry.update();
        sleep(2000);

        int traverseInitialPosition = traverse.getCurrentPosition();
        telemetry.addData("Traverse Initial Position: ", traverseInitialPosition);
        traversing(0.7, (int) ((arm_ticks_per_rev/4) + traverseInitialPosition));
        sleep(2000);

        int traverseHalfRevolutionPosition = traverse.getCurrentPosition();
        telemetry.addData("Traverse Half Revolution Position: ", traverseHalfRevolutionPosition);
        telemetry.update();
        traversing(-0.7, traverseInitialPosition);
        sleep(3000);

        int expansionInitialPosition = expansion.getCurrentPosition();
        telemetry.addData("Traverse Initial Position: ", expansionInitialPosition);
        telemetry.update();
        expand(0.5, (int) ((expand_ticks_per_rev) + expansionInitialPosition));
        sleep(2000);

        int expansionHalfRevolutionPosition = expansion.getCurrentPosition();
        telemetry.addData("Traverse Half Revolution Position: ", expansionHalfRevolutionPosition);
        telemetry.update();
        expand(-0.5, expansionInitialPosition);
        sleep(3000);

        openClaw();
        closeClaw();
    }

    public void stopArmMovement() {
        expansion.setPower(0);
        rotation.setPower(0);
        traverse.setPower(0);
    }

    public void rotate(double power, int distance) {
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotation.setTargetPosition(distance);

        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rotation.setPower(power);

        while (rotation.isBusy()) {

        }

        stopArmMovement();
        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void expand(double power, int distance) {
        expansion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        expansion.setTargetPosition(distance);

        expansion.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        expansion.setPower(power);

        while (expansion.isBusy()) {

        }

        stopArmMovement();
        expansion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void traversing(double power, int distance) {
        traverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        traverse.setTargetPosition(distance);

        traverse.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        traverse.setPower(power);
        while (traverse.isBusy()) {

        }

        stopArmMovement();
        traverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int distance(double angle) {
        ticks = (int) (angle * ticks_per_angle);
        return ticks;
    }

    public void closeClaw() {
        clamp.setPosition(0.3);
    }

    public void openClaw() {
        clamp.setPosition(0.6);
    }
}