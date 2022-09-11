package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DiverTwoWheelTest extends LinearOpMode {

    private TwoWheel driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new TwoWheel(
                hardwareMap.get(DcMotor.class, "left"),
                hardwareMap.get(DcMotor.class, "right"),
                hardwareMap.get(BNO055IMU.class, "imu")
        );

        waitForStart();

        while (opModeIsActive()) {

            // gamepad controls
            if (gamepad1.x) {
                driveTrain.brake(0);
            } else {
                driveTrain.drive(gamepad1.right_trigger - gamepad1.left_trigger, gamepad1.left_stick_x);
            }

            // telemetry test for localization
            telemetry.addData("x position", driveTrain.getPosition()[0]);
            telemetry.addData("y position", driveTrain.getPosition()[1]);
            telemetry.update();
        }
    }
}
