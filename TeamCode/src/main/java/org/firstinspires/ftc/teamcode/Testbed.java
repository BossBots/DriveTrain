package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Testbed extends LinearOpMode {

    public DcMotor motor;

    public Servo servo1;
    public boolean oldPress = false;
    public boolean newPress = false;
    public boolean isClosed = false;

    public CRServo servo2;

    public BNO055IMU imu;
    public Parameters params;
    public Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        params = new Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("status", "running");

            // IMU
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES);
            telemetry.addData("x orientation", angles.firstAngle);
            telemetry.addData("y orientation", angles.secondAngle);
            telemetry.addData("z orientation", angles.thirdAngle);

            // motor
            if (gamepad1.x) {
                motor.setPower(0);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                motor.setPower(-gamepad1.right_stick_y);
            }
            telemetry.addData("encoder position", motor.getCurrentPosition());

            // standard scale servo
            newPress = gamepad1.a;
            if (newPress && !oldPress) {
                isClosed = !isClosed;
            }
            oldPress = newPress;
            telemetry.addData("closed", isClosed);
            if (isClosed) {
                servo1.setPosition(1);
            } else {
                servo1.setPosition(0);
            }

            // cr servo
            if (gamepad1.right_bumper) {
                servo2.setPower(1);
            } else if (gamepad1.left_bumper) {
                servo2.setPower(-1);
            } else {
                servo2.setPower(0);
            }

            telemetry.update();

        }


    }
}
