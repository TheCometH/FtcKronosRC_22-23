package org.firstinspires.ftc.robotcontroller.internal;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp
public class drive extends OpMode {

    private Motor fL, fR, bL, bR;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private BNO055IMU imu;

    @Override
    public void init() {
        /* instantiate motors */
        BNO055IMU imu;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);


        fL = hardwareMap.get(Motor.class, "leftFront");
        bL = hardwareMap.get(Motor.class, "leftRear");
        bR = hardwareMap.get(Motor.class, "rightRear");
        fR = hardwareMap.get(Motor.class, "rightFront");

        fL.setInverted(true);
        bL.setInverted(true);
        bR.setInverted(false);
        fR.setInverted(false);

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        drive = new MecanumDrive(fL, fR, bL, bR);
        driverOp = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        drive.driveRobotCentric(

                deadband(driverOp.getLeftX(), 0.1),
                deadband(driverOp.getLeftY(), 0.1),
                deadband(driverOp.getRightY(), 0.1)
        );

//        drive.driveFieldCentric(
//                driverOp.getLeftX(),
//                driverOp.getLeftY(),
//                driverOp.getRightY(),
//                imu.getAngularOrientation().firstAngle
//
//        );
    }
    public double deadband(double value, double deadband){
        if(value > deadband)
            return value;
        return 0;
    }

}

//     implementation 'org.ftclib.ftclib:core:2.0.1' // core

