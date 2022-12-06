package org.firstinspires.ftc.teamcode.auto.cameradetection;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Autonomous
public class TestOpenCV extends LinearOpMode {
    CameraDetectionV2 camera = new CameraDetectionV2();
    int park = 0;
    @Override
    public void runOpMode() throws InterruptedException {
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
            park = camera.check();
        }
        telemetry.addLine("parking spot: " + park);
        telemetry.update();
        sleep(20);
    }
}