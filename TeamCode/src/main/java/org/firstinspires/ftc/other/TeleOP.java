package org.firstinspires.ftc.other;/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;


/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "TeleO", group = "Example")
@Disabled
public class TeleOP extends OpMode {

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    DcMotor frMotor, flMotor, brMotor, blMotor, wheel, launcherR, launcherL;
    CRServo collector;
    Servo launcherAngle;
    Servo launcherAngleR;
    Servo collectorHinge;
    BNO055IMU imu;
    Orientation gyroAngles;
    ColorSensor colorSensor;
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_REV = 8192; // CPR for REV Through Bore Encoders
    final double WHEEL_DIAMETER = 1.49606; //in inches, 38mm for odometry aluminum omni wheels
    double COUNTS_PER_INCH = COUNTS_PER_REV / (WHEEL_DIAMETER * 3.1415);

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    double frPower=0, flPower=0, brPower=0, blPower=0, collectorPower, launchPower;
    double maxMotorPower=0;
    double driveSpeed = 0;
    double driveRotation = 0;
    double a, b, x, y, joystickAngle, joystickAngle360;
    double desiredRobotHeading;
    int rotations = 0;
    @Override
    public void init() {
        frMotor = hardwareMap.dcMotor.get("frontright");
        flMotor = hardwareMap.dcMotor.get("frontleft");
        brMotor = hardwareMap.dcMotor.get("backright");
        blMotor = hardwareMap.dcMotor.get("backleft");
        collector = hardwareMap.crservo.get("collector");
        launcherR = hardwareMap.dcMotor.get("launcherR");
        launcherL = hardwareMap.dcMotor.get("launcherL");
        wheel = hardwareMap.dcMotor.get("wheel");
        launcherAngle = hardwareMap.get(Servo.class, "ServoL");
        launcherAngleR = hardwareMap.get(Servo.class, "ServoR");

        collectorHinge = hardwareMap.get(Servo.class, "collectorHinge");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75, 111, 8.5, 0.0);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("1", "Integrated Heading: " + getIntegratedHeading());
        telemetry.addData("2", "heading: " + gyroAngles.firstAngle);
        telemetry.addData("1 Right Motor Pos", frMotor.getCurrentPosition());
        telemetry.addData("2 Left Motor Pos", flMotor.getCurrentPosition());
    }


    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {

        telemetry.addData("R", colorSensor.red() );
        telemetry.addData("G", colorSensor.green() );
        telemetry.addData("B", colorSensor.blue() );
        telemetry.addData("A", colorSensor.alpha() );
        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        y = Range.clip(-gamepad1.right_stick_y, -1, 1);
        x = Range.clip(gamepad1.right_stick_x, -1, 1);
        joystickAngle = Math.atan2(-x, y);
        joystickAngle360 = joystickAngle >= 0 ? joystickAngle : (2 * Math.PI) + joystickAngle;

        driveSpeed = Range.clip(Math.sqrt(y * y + x * x), -1, 1);
        if (gamepad1.right_trigger > .05 || gamepad1.left_trigger > .05) {
            driveRotation = gamepad1.left_trigger - gamepad1.right_trigger;
            desiredRobotHeading = getIntegratedHeading();
        } else if (Math.abs(desiredRobotHeading - getIntegratedHeading()) > 5) {
            driveRotation = (desiredRobotHeading - getIntegratedHeading()) / (Math.abs(desiredRobotHeading - getIntegratedHeading())) * .5;
        } else {
            driveRotation = 0;
        }

        if (gamepad2.right_bumper) //launcher, right bumper, where you can shoot; runs both motors for wheels in launcher
        {
            launcherR.setPower(.7); // not set value
            launcherL.setPower(.7);
        }
        else
        {
            launcherR.setPower(0); // launcher motors off
            launcherL.setPower(0);
        }
        if (gamepad1.left_bumper) //launcher, right bumper, where you can shoot; runs both motors for wheels in launcher
        {
            collectorHinge.setPosition(.7); // not set value
        }
        if (gamepad2.y)
        {
            launcherAngle.setPosition(.3); //value not set position will be figured out with angles
        }
        /*if (gamepad1.a) {
            collector.setPower(1);
            //collectorPower = 1;
        } else {
            collector.setPower(0);
            //collectorPower = 0;
        }

        if (gamepad1.b) {
            launcher.setPower(1);
        } else {
            launcher.setPower(0);
        }*/

        flPower = Math.cos(joystickAngle360 - Math.toRadians(desiredRobotHeading) + Math.PI / 4);
        frPower = Math.sin(joystickAngle360 - Math.toRadians(desiredRobotHeading) + Math.PI / 4);
        blPower = Math.sin(joystickAngle360 - Math.toRadians(desiredRobotHeading) + Math.PI / 4);
        brPower = Math.cos(joystickAngle360 - Math.toRadians(desiredRobotHeading) + Math.PI / 4);
        maxMotorPower = Math.max(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.abs(blPower)), Math.abs(brPower));

        //Ratio the powers for direction
        flPower = flPower / maxMotorPower;
        frPower = frPower / maxMotorPower;
        blPower = blPower / maxMotorPower;
        brPower = brPower / maxMotorPower;

        flPower = driveSpeed * flPower - driveRotation;
        frPower = driveSpeed * frPower + driveRotation;
        blPower = driveSpeed * blPower - driveRotation;
        brPower = driveSpeed * brPower + driveRotation;

        maxMotorPower = Math.max(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.abs(blPower)), Math.abs(brPower));

        if (Math.abs(maxMotorPower) > 1) {
            flPower = flPower / maxMotorPower;
            frPower = frPower / maxMotorPower;
            blPower = blPower / maxMotorPower;
            brPower = brPower / maxMotorPower;
        } else if(Math.abs(maxMotorPower) < .03) {
            flPower = 0;
            frPower = 0;
            blPower = 0;
            brPower = 0;
        }

        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }


    @Override
    public void stop() { }

    private double getIntegratedHeading() {
        if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) > 200) {
            rotations++;
        }
        else if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) < -200) {
            rotations--;
        }

        return (rotations * 360 + gyroAngles.firstAngle);
    }

}
