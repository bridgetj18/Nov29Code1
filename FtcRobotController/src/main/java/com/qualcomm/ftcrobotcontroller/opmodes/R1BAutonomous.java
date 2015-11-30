package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by bridgetj18 on 11/29/15.
 */
public class R1BAutonomous extends LinearOpMode{
    DcMotor right;
    DcMotor left;
    double speed = 1;
    double targetHeading = 0.0;
    double gain = 0.1;
    double steeringError;
    double leftPower;
    double rightPower;
    int currentHeading = 0;
    double steeringAdjustment = 0;
    int a = 0;
    int b = 0;
    int c = 0;
    int d = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        right = hardwareMap.dcMotor.get("m1");
        left = hardwareMap.dcMotor.get("m2");
        right.setDirection(DcMotor.Direction.REVERSE);
        GyroSensor sensorGyro;
        // write some device information (connection info, name and type)
        // to the log file.
        hardwareMap.logDevices();
        // get a reference to our GyroSensor object.
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        // calibrate the gyro.
        sensorGyro.calibrate();
        // wait for the start button to be pressed.
        waitForStart();
        // make sure the gyro is calibrated.
        while (sensorGyro.isCalibrating()) {
            Thread.sleep(50);
        }
        while (opModeIsActive()) {
            // Wait for the start button to be pressed
            waitForStart();

            currentHeading = sensorGyro.getHeading();
            if (currentHeading > 180){
                currentHeading -= 360;
            }

            steeringError = currentHeading - targetHeading;
            steeringAdjustment = steeringError * gain;
            rightPower = (speed - steeringAdjustment);
            leftPower = (speed + steeringAdjustment);

            //Set the motors to drive the robot forward
            left.setPower(0.5);
            right.setPower(0.5);

            //travel for b seconds
            sleep(b);

            //Set the motors to turn the robot right
            currentHeading = -135;

            //travel for d seconds
            sleep(d);

            //Stop the robot
            left.setPower(0);
            right.setPower(0);
        }
    }
}
