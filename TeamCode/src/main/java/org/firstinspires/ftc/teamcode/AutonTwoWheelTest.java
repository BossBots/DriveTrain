package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutonTwoWheelTest extends LinearOpMode {

    private TwoWheel driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new TwoWheel(
                hardwareMap.get(DcMotor.class, "left"),
                hardwareMap.get(DcMotor.class, "right"),
                hardwareMap.get(BNO055IMU.class, "imu")
        );

        waitForStart();

        if (opModeIsActive()) {

            driveTrain.forward(0.5, 1.5, 5000);
            driveTrain.rotate(-0.5, 90, 5000);

        }
    }
}
