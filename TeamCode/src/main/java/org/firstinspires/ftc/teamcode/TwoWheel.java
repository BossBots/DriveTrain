package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class TwoWheel {

    // motor variables
    private double leftPower = 0;
    private double rightPower = 0;
    private DcMotor left;
    private DcMotor right;

    // IMU variables
    private Orientation angles;
    private BNO055IMU imu;
    private BNO055IMU.Parameters params;

    // localization variables
    private int counts = 0;
    private int[] initEncoderPos;
    private int[] finalEncoderPos;
    private double displacement;
    private double[] position;
    private final double CONVERSION = 2. * Math.PI * 0.0254 / 480.;     // units: meters per tick

    // constructor
    public TwoWheel(DcMotor initLeft, DcMotor initRight, BNO055IMU initIMU) {
        left = initLeft;
        right = initRight;
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        imu = initIMU;
        params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);

        position = new double[2];
    }

    public double getAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return angles.thirdAngle;
    }

    public double[] getPosition() {return position;}

    public void setMode(DcMotor.RunMode mode) {
        right.setMode(mode);
        left.setMode(mode);
    }

    public void brake(long dur) {
        drive(0, 0);
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < dur) {
            left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void forward(double pwr, double dist, long maxDur) {
        double initAngle = getAngle();
        double[] initPos = new double[] {position[0], position[1]};
        long start = System.currentTimeMillis();
        while (Math.sqrt(Math.pow(position[0] - initPos[0], 2) + Math.pow(position[1] - initPos[1], 2)) < dist &&
            System.currentTimeMillis() - start < maxDur) {
            if (Math.abs(getAngle() - initAngle) < 5) {
                drive(pwr, 0);
            } else if (getAngle() - initAngle > 5) {
                drive(pwr, pwr/2);
            } else {
                drive(pwr, -pwr/2);
            }
        }
        brake(125);
    }

    public void rotate(double pwr, double targetAngle, long maxDur) {
        double currentAngle = getAngle();
        long start = System.currentTimeMillis();
        if (targetAngle != 180) {
            while (Math.abs(currentAngle - targetAngle) > 3 && System.currentTimeMillis() - start < maxDur) {
                drive(0, pwr);
                currentAngle = getAngle();
            }
        } else {
            while (Math.abs(targetAngle - Math.abs(currentAngle)) > 3 && System.currentTimeMillis() - start < maxDur) {
                currentAngle = getAngle();
            }
        }
        brake(125);
    }

    public void localize() {
        if (counts == 0) {
            initEncoderPos = new int[] {left.getCurrentPosition(), right.getCurrentPosition()};
        }
        finalEncoderPos = new int[] {left.getCurrentPosition(), right.getCurrentPosition()};
        double angle = getAngle();
        displacement = CONVERSION * (-(finalEncoderPos[0] - initEncoderPos[0]) + (finalEncoderPos[1] - initEncoderPos[1])) / 2.;
        position[0] += displacement * Math.cos(angle * Math.PI / 180.);
        position[1] += displacement * Math.sin(angle * Math.PI / 180);
        initEncoderPos[0] = finalEncoderPos[0];
        initEncoderPos[1] = finalEncoderPos[1];
        counts++;
    }

    public void drive(double linear, double rotational) {

        localize();

        leftPower = linear + rotational;
        rightPower = linear - rotational;

        if (leftPower > 1) {
            leftPower = 1;
        } else if (leftPower < -1) {
            leftPower = -1;
        }

        if (rightPower > 1) {
            rightPower = 1;
        } else if (rightPower < -1) {
            rightPower = -1;
        }

        left.setPower(-leftPower);
        right.setPower(rightPower);
    }


}
