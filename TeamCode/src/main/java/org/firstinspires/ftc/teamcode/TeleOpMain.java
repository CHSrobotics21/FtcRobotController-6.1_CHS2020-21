
package org.firstinspires.ftc.teamcode;/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this listF
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;


/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Standard Tele Op", group = "Example")
//@Disabled
public class TeleOpMain extends OpMode {

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    DcMotor frMotor, flMotor, brMotor, blMotor, collectorWheel, collector;
    Servo launcherAngle, launcherAngleR;
    DcMotorEx launcherR, launcherL;
    BNO055IMU imu;
    Orientation gyroAngles;
    ColorSensor distanceColor;
    Servo wobbleArmGripL, wobbleArmGripR, ringStopper;
    CRServo wobbleArmHingeL, wobbleArmHingeR;
    DcMotor verticalLeft, verticalRight, horizontal;
    DistanceSensor intakeDistanceSensor, outDistanceSensor, ringStopperSensor;

    double frPower=0, flPower=0, brPower=0, blPower=0, collectorPower, launchPower;
    double maxMotorPower=0;
    double driveSpeed = 0;
    double driveRotation = 0;
    double a, b, x, y, joystickAngle, joystickAngle360;
    double desiredRobotHeading;
    int rotations = 0, wobbleEncoderCounts = 0;
    final double COUNTS_PER_REV = 8192; // CPR for REV Through Bore Encoders
    final double WHEEL_DIAMETER = 2.3622; //in inches, 38mm for odometry aluminum omni wheels
    double COUNTS_PER_INCH = COUNTS_PER_REV / (WHEEL_DIAMETER * 3.1415);
    final double CPRCollectorWheel = 288;
    final double CollectorWheelDiameter = 5;
    double CPICollectorWheel = CPRCollectorWheel/(CollectorWheelDiameter*3.1415);
    OdometryGlobalCoordinatePosition globalPositionUpdate;

    Thread positionThread;
    boolean buttonPressed = false;
    boolean AIlaunch = false;
    double servoAdd = .4;
    boolean stopSensor = false;
    boolean isWheelRunning = false;
    boolean islaunchRunning= false;
    boolean lockDrive = false;
    boolean isCollectorWheel = false;
    boolean launchToggle = false;
    boolean gripToggle = false;
    boolean collectorToggle = false;
    boolean collectorCanToggle = true;
    boolean launcherCanToggle = true;
    boolean gripCanToggle = true;
    boolean isCollectorRunning = false;
    boolean collectorReverseCanToggle = false, collectorReverseToggle = false;
    int rings = 0;
    boolean topCurrent = false;
    boolean topPrevious = false;

    boolean ringStopperToggle = false, ringStopperCanToggle = true;
    boolean isGoToPosition = false;
    private Double startX;
    private Double startY;
    public Double startOrientation;

    boolean robotPerspective = false;
    double fieldReference = 0.0;
    boolean perspectiveToggle = false, perspectiveCanToggle = false;

    boolean isWobble = false;

    @Override
    public void init() {
        //distanceColor = hardwareMap.colorSensor.get("distanceColor");
        frMotor = hardwareMap.dcMotor.get("frontright");
        flMotor = hardwareMap.dcMotor.get("frontleft");
        brMotor = hardwareMap.dcMotor.get("backright");
        blMotor = hardwareMap.dcMotor.get("backleft");
        collector = hardwareMap.dcMotor.get("collector");

        ringStopperSensor = hardwareMap.get(DistanceSensor.class,"ringStopperSensor");
        distanceColor = hardwareMap.get(ColorSensor.class, "ringStopperSensor");

        launcherR = hardwareMap.get(DcMotorEx.class,"launcherR");
        launcherL = hardwareMap.get(DcMotorEx.class,"launcherL");
        launcherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        collectorWheel = hardwareMap.dcMotor.get("wheel");
        launcherAngle = hardwareMap.get(Servo.class, "ServoL");
       launcherAngleR = hardwareMap.get(Servo.class, "ServoR");

        wobbleArmGripL = hardwareMap.servo.get("GripL");
        wobbleArmGripR = hardwareMap.servo.get("GripR");
        ringStopper = hardwareMap.servo.get("ringStopper");
        wobbleArmHingeL = hardwareMap.crservo.get("HingeL");
        wobbleArmHingeR = hardwareMap.crservo.get("HingeR");
        intakeDistanceSensor = hardwareMap.get(DistanceSensor.class, "intake");


        verticalLeft = hardwareMap.dcMotor.get("backleft");
        verticalRight = hardwareMap.dcMotor.get("frontleft");
        horizontal = hardwareMap.dcMotor.get("frontright");

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ringStopper.setPosition(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Status", "Initialized");
        File TeleOpStartingPos = AppUtil.getInstance().getSettingsFile("TeleOpStartingPos.txt");
        String fileContents = ReadWriteFile.readFile(TeleOpStartingPos).trim();
        String[] array = fileContents.split(" ");
        startX = Double.parseDouble(array[0]);
        startY = Double.parseDouble(array[1]);
        startOrientation = Double.parseDouble(array[2]);


        telemetry.addData("StartingPostionX", startX);
        telemetry.addData("StartingPostionY", startY);
        telemetry.addData("StartingOrientation", startOrientation);
        telemetry.update();

    }


    @Override
    public void init_loop() {
        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("1", "Integrated Heading: " + getIntegratedHeading());
        telemetry.addData("ring sensor data: ", ringStopperSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("ring sensor ALPHA: ", distanceColor.alpha());
        telemetry.addData("ring sensor RED: ", distanceColor.red());
        telemetry.addData("ring sensor BLUE: ", distanceColor.blue());
        telemetry.addData("ring sensor GREEN: ", distanceColor.green());
//        telemetry.addData("2", "heading: " + globalPositionUpdate.returnOrientation());
//        telemetry.addData("StartingPostionX", globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);
//        telemetry.addData("StartingPostionY", globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH);
        // telemetry.addData("1 Right Motor Pos", frMotor.getCurrentPosition());
        // telemetry.addData("2 Left Motor Pos", flMotor.getCurrentPosition());


        telemetry.addData("StartingPostionX", startX);
        telemetry.addData("StartingPostionY", startY);
        telemetry.addData("StartingOrientation", startOrientation);
        telemetry.update();
    }


    @Override
    public void start() {
        runtime.reset();
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75, startX, startY, startOrientation);
        positionThread = new Thread(globalPositionUpdate);

        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseLeftEncoder();
    }


    @Override
    public void loop() {

        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if(gamepad2.left_bumper){
            collectorWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            collectorWheel.setPower(-1);
            isWheelRunning = true;
        }
//        else if(intakeDistanceSensor.getDistance(DistanceUnit.INCH)<6.5&&intakeDistanceSensor.getDistance(DistanceUnit.INCH)>0 &&rings < 3){
//            collectorWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            collectorWheel.setPower(-1);
//            timer.reset();
//        }
        else if(gamepad2.right_bumper){
            collectorWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            collectorWheel.setPower(1);
            isWheelRunning = true;
        }
        else{
            collectorWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            collectorWheel.setPower(0);
            isWheelRunning = false;
        }
//
        if(timer.time()>2&&timer.time()<2.05){
            rings++;
            collectorWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            collectorWheel.setPower(0);
        }
        topPrevious = topCurrent;
        topCurrent = ringStopperSensor.getDistance(DistanceUnit.CM)<4.6&&ringStopperSensor.getDistance(DistanceUnit.CM)>0;
        if (topPrevious && !topCurrent){
            rings--;
        }
        if (ringStopperSensor.getDistance(DistanceUnit.CM)<4.6&&ringStopperSensor.getDistance(DistanceUnit.CM)>0 && islaunchRunning){
            moveCollectorWheel();
            isWheelRunning = false;
        }
//        else if(gamepad2.left_trigger>.05){
//            isWheelRunning = true;
//            collectorwheelthread.moveCollectorWheel(8, false);
//            collectorWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            isWheelRunning = false;
//        }
//        if(gamepad2.right_bumper&&!isWheelRunning){
//            isCollectorWheel = true;
//            collectorWheel.setPower(-.9);
//        }

        if (gamepad2.dpad_down){
            launcherAngleR.setPosition(.25);
            launcherAngle.setPosition(.25);
        }
        if (gamepad2.dpad_left){
            launcherAngleR.setPosition(.35);
            launcherAngle.setPosition(.35);
        }
        if (gamepad2.dpad_up){
            launcherAngleR.setPosition(.42);
            launcherAngle.setPosition(.42);
        }
        if(gamepad2.dpad_right){
            if(ringStopperCanToggle) //make sure that the code doesn't just toggle the thing every iteration as long as the trigger's held
            {
                ringStopperCanToggle=false;
                //if the collector is currently running, run this code to turn it off:
                if(ringStopperToggle)
                {
                    ringStopper.setPosition(0); //turn off the collector motor
                    ringStopperToggle=false; //remember that the collector motor has been turned off
                }
                //if the collector isn't currently running, run this code to turn it on:
                else
                {
                    ringStopper.setPosition(1); //turn on the collector motor
                    ringStopperToggle=true; //remember that the collector motor has been turned on
                }
            }
        }
        else
        {
            ringStopperCanToggle=true;
        }
        if(gamepad2.right_trigger > .05)
        {
            if(collectorCanToggle) //make sure that the code doesn't just toggle the thing every iteration as long as the x is held
            {
                collectorCanToggle=false;
                //if the launcher is currently running, run this code to turn it off:
                if(collectorToggle)
                {
                    collector.setPower(0); //turn off the launcher motor
                    collectorToggle=false; //remember that the launcher motor has been turned off
                }
                //if the launcher isn't currently running, run this code to turn it on:
                else
                {
                    collector.setPower(-.9); //turn on the launcher motor
                    collectorToggle =true; //remember that the launcher motor has been turned on
                }
            }
        }
        else
        {
            collectorCanToggle=true;
        }
        if(gamepad2.left_trigger>.05){
            if(collectorReverseCanToggle) //make sure that the code doesn't just toggle the thing every iteration as long as the trigger's held
            {
                collectorReverseCanToggle=false;
                //if the collector is currently running, run this code to turn it off:
                if(collectorReverseToggle)
                {
                    collector.setPower(0); //turn off the collector motor
                    collectorReverseToggle=false; //remember that the collector motor has been turned off
                }
                //if the collector isn't currently running, run this code to turn it on:
                else
                {
                    collector.setPower(.9); //turn on the collector motor
                    collectorReverseToggle=true; //remember that the collector motor has been turned on
                }
            }
        }
        else
        {
            collectorReverseCanToggle=true;
        }
         if(gamepad2.x)
         {
             if(launcherCanToggle) //make sure that the code doesn't just toggle the thing every iteration as long as the x is held
            {
             launcherCanToggle=false;
             //if the launcher is currently running, run this code to turn it off:
             if(launchToggle)
             {
                 launchSetZero(); //turn off the launcher motor
                 ringStopper.setPosition(0);
                 launchToggle=false; //remember that the launcher motor has been turned off
             }
             //if the launcher isn't currently running, run this code to turn it on:
             else
             {
                 launch(); //turn on the launcher motor
                 // change position but it is automatic that when the launcher is on, the ringstopper servo will come up and out of the way
                    ringStopper.setPosition(1);
                 
                 launchToggle=true; //remember that the launcher motor has been turned on
             }
            }
         }
         else
         {
             launcherCanToggle=true;
         }
//        else if(intakeDistanceSensor.getDistance(DistanceUnit.INCH)<6.5||intakeDistanceSensor.getDistance(DistanceUnit.INCH)>20&&!stopSensor){
//            collector.setPower(.7);
//            moveCollectorWheel();
//            collectorWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            collectorWheel.setTargetPosition(0);
//        }
//        else if(AIlaunch){
//            collectorWheel.setPower(0);
//            isCollectorWheel = false;
//            AIlaunch = false;
//        }
//        else{
//
//            collectorWheel.setPower(0);
//            isCollectorWheel = false;
//
//        }

        if(gamepad2.b)
        {
            if(gripCanToggle) //make sure that the code doesn't just toggle the thing every iteration as long as the x is held
            {
                gripCanToggle=false;
                //if the launcher is currently running, run this code to turn it off:
                if(gripToggle)
                {
                    grip(false); //turn off the launcher motor
                    gripToggle=false; //remember that the launcher motor has been turned off
                }
                //if the launcher isn't currently running, run this code to turn it on:
                else
                {
                    grip(true); //grip
                    isWobble = true;
                    gripToggle=true; //remember that the launcher motor has been turned on
                }
            }
        }
        else
        {
            gripCanToggle=true;
        }

        if(gamepad2.x)
        {
            if(launcherCanToggle) //make sure that the code doesn't just toggle the thing every iteration as long as the x is held
            {
                launcherCanToggle=false;
                //if the launcher is currently running, run this code to turn it off:
                if(launchToggle)
                {
                    launchSetZero(); //turn off the launcher motor
                    launchToggle=false; //remember that the launcher motor has been turned off
                }
                //if the launcher isn't currently running, run this code to turn it on:
                else
                {
                    launch(); //turn on the launcher motor
                    launchToggle=true; //remember that the launcher motor has been turned on
                }
            }
        }
        else
        {
            launcherCanToggle=true;
        }

        if(gamepad2.y)
        {
            hinge(false);
        }
        else if(gamepad2.a)
        {
            hinge(true);
        }
        else{
            wobbleArmHingeR.setPower(0);
            wobbleArmHingeL.setPower(0);
        }

        if(islaunchRunning&&isWheelRunning){
            lockDrive = true;
        }
        else{
            lockDrive = false;
        }

        if (gamepad1.right_trigger > .05 || gamepad1.left_trigger > .05) {
            driveRotation = gamepad1.left_trigger - gamepad1.right_trigger;
            desiredRobotHeading = getIntegratedHeading();
        } else if (Math.abs(desiredRobotHeading - getIntegratedHeading()) > 5) {
            driveRotation = (desiredRobotHeading - getIntegratedHeading()) / (Math.abs(desiredRobotHeading - getIntegratedHeading())) * .05;
        }
        else {
            driveRotation = 0;
        }
        if(gamepad1.right_bumper){
            if(perspectiveCanToggle) //make sure that the code doesn't just toggle the thing every iteration as long as the trigger's held
            {
                perspectiveCanToggle=false;
                if(perspectiveToggle)
                {//if the motor is currently running, turn it off
                    robotPerspective = true; //turn off the motor
                    perspectiveToggle=false; //remember that the collector motor has been turned off
                }
                else
                {//if the motor isn't currently running, turn it on
                    robotPerspective = false; //turn on motor
                    perspectiveToggle=true; //remember that the motor has been turned on
                }
            }
        }
        else
        {
            perspectiveCanToggle=true;
        }
        if (robotPerspective) { //Controls are mapped to the robot perspective
            fieldReference = 0;
            //Positive values for x axis are joystick right
            //Positive values for y axis are joystick down
            y = Range.clip(-gamepad1.right_stick_y,-1,1);
            x = Range.clip(-gamepad1.right_stick_x,-1,1);
            joystickAngle = Math.atan2(x,y);
        } else {   //Controls are mapped to the field
            fieldReference = desiredRobotHeading;
            //Positive values for x axis are joystick right
            //Positive values for y axis are joystick down
            y = Range.clip(gamepad1.right_stick_x,-1,1);
            x = Range.clip(-gamepad1.right_stick_y,-1,1);
            joystickAngle = Math.atan2(x,y);
            // joystickAngle360 = joystickAngle >= 0 ? joystickAngle : (2*Math.PI) + joystickAngle;
        }
        joystickAngle360 = joystickAngle >= 0 ? joystickAngle : (2*Math.PI) + joystickAngle;
        driveSpeed = Range.clip(Math.sqrt(y * y + x * x), -1, 1);

        flPower = Math.cos(joystickAngle360 - Math.toRadians(fieldReference) + Math.PI / 4);
        frPower = Math.sin(joystickAngle360 - Math.toRadians(fieldReference) + Math.PI / 4);
        blPower = Math.sin(joystickAngle360 - Math.toRadians(fieldReference) + Math.PI / 4);
        brPower = Math.cos(joystickAngle360 - Math.toRadians(fieldReference) + Math.PI / 4);

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
        } else if (Math.abs(maxMotorPower) < .03) {
            flPower = 0;
            frPower = 0;
            blPower = 0;
            brPower = 0;
        }

        if(!lockDrive)
        {

            if(gamepad1.dpad_up){
                goToPositionSlowDown(111, 70, .7, 0); // go to shooting position
            }
            if(gamepad1.x){
                goToAngle(111,70,.7,-16, 1 );//angle robot using IMU
                //powershot 1
            }
            else if(gamepad1.a){
                goToAngle(111,70,.7,-20.5, 1 );
            }
            else if(gamepad1.b){
                goToAngle(111,70,.7,-25, 1 ); //powershot 3
            }
            else if(gamepad1.y){
                goToPositionSlowDown(111,60,.6,0);// launching high tower
            }
            else if(gamepad1.left_bumper){
                flMotor.setPower(flPower*.5);
                frMotor.setPower(frPower*.5);
                blMotor.setPower(blPower*.5);
                brMotor.setPower(brPower*.5);
            }
            else{
                flMotor.setPower(flPower);
                frMotor.setPower(frPower);
                blMotor.setPower(blPower);
                brMotor.setPower(brPower);
            }
        }

        else{
                flMotor.setPower(0);
                frMotor.setPower(0);
                blMotor.setPower(0);
                brMotor.setPower(0);
        }

//        if (gamepad1.b){
//
//            launcherAngleR.setPosition(.4);
//            launcherAngle.setPosition(.4);
//
//        }
//        if(gamepad1.a){
//
//            launcherAngleR.setPosition(.5);
//            launcherAngle.setPosition(.5);
//
//        }
//        if (ringStopperSensor.getDistance(DistanceUnit.CM)<2){
//            collectorwheelthread.moveCollectorWheel(0);
//            isWheelRunning = false;
//        }

//        telemetry.addData("rings: ",rings);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Wobble counts", brMotor.getCurrentPosition());
        telemetry.addData("StartingPostionX", globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);
        telemetry.addData("StartingPostionY", globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH);
//        telemetry.addData("intake distance Sensor: ", String.format("%.01f cm",intakeDistanceSensor.getDistance(DistanceUnit.CM)));
//        telemetry.addData("sensor timer: ", timer.time());
//        telemetry.addData("previous: ", topPrevious);
//        telemetry.addData("current", topCurrent);
//        telemetry.addData("Distance ring stopper: ", ringStopperSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Launcher angle: ", launcherAngleR.getPosition());
        telemetry.addData("Launcher angle: ", launcherAngle.getPosition());
        telemetry.addData("Robot angle: ", getIntegratedHeading());
        telemetry.addData("launcherL velocity: ", launcherL.getVelocity());
        telemetry.addData("launcherR velocity: ", launcherR.getVelocity());
        telemetry.addData("ring sensor data: ", ringStopperSensor.getDistance(DistanceUnit.CM));
    }



    public boolean goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError ) {
        targetXPosition *= COUNTS_PER_INCH;
        isGoToPosition = true;
        targetYPosition *= COUNTS_PER_INCH;
        allowableDistanceError *= COUNTS_PER_INCH;
        double blPower = 0; // motor speed
        double brPower = 0; // motor speed
        double flPower = 0; // motor speed
        double frPower = 0; // motor speed
        double pivotCorrectionAdj = .01; // constant to scale down pivot correction angle to work with setting powers for mecanum drive motors
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        if (distance > allowableDistanceError) { //correct heading too
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double robotMovmentXComponent = calculateX(robotMovementAngle - globalPositionUpdate.returnOrientation(), robotPower);
            double robotMovmentYComponent = calculateY(robotMovementAngle - globalPositionUpdate.returnOrientation(), robotPower);
            double pivotCorrection = (desiredRobotOrientation - globalPositionUpdate.returnOrientation()) * pivotCorrectionAdj;
            //double[] powers = {robotMovmentYComponent, robotMovmentYComponent, robotMovmentYComponent, robotMovmentYComponent};//array for powers
            blPower = robotMovmentYComponent - robotMovmentXComponent + pivotCorrection;
            flPower = robotMovmentYComponent + robotMovmentXComponent + pivotCorrection;
            brPower = robotMovmentYComponent + robotMovmentXComponent - pivotCorrection;
            frPower = robotMovmentYComponent - robotMovmentXComponent - pivotCorrection;
            //set powers to motors to move
            double maxMotorPower = Math.max(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.abs(blPower)), Math.abs(brPower));

            if (Math.abs(maxMotorPower) > 1) {
                flPower = (flPower / maxMotorPower) * robotPower;
                frPower = (frPower / maxMotorPower) * robotPower;
                blPower = (blPower / maxMotorPower) * robotPower;
                brPower = (brPower / maxMotorPower) * robotPower;
            } else if (Math.abs(maxMotorPower) < .03) {
                flPower = 0;
                frPower = 0;
                blPower = 0;
                brPower = 0;
            }
            flMotor.setPower(flPower);
            frMotor.setPower(frPower);
            blMotor.setPower(blPower);
            brMotor.setPower(brPower);
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("XComponent: ", robotMovmentXComponent / .9);
            telemetry.addData("YComponent: ", robotMovmentYComponent / .9);
            telemetry.addData("Pivot Correction: ", pivotCorrection);
            telemetry.addData("Gyro orientation: ", imu.getAngularOrientation().firstAngle);
            telemetry.update();
            isGoToPosition = true;
            return false;
        }
        else{
            flMotor.setPower(0);
            frMotor.setPower(0);
            blMotor.setPower(0);
            brMotor.setPower(0);
            isGoToPosition = false;
            return true;

        }
    }
    public void goToPositionSlowDown(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation){
        if (
        goToPosition(targetXPosition,targetYPosition,robotPower,desiredRobotOrientation,8)) {}
        if (
        goToPosition(targetXPosition,targetYPosition,robotPower-.3,desiredRobotOrientation,1)) {}
        
    }
    public boolean goToAngle(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableAngleError ) {
        targetXPosition *= COUNTS_PER_INCH;
        isGoToPosition = true;
        targetYPosition *= COUNTS_PER_INCH;
        double blPower = 0; // motor speed
        double brPower = 0; // motor speed
        double flPower = 0; // motor speed
        double frPower = 0; // motor speed
        double pivotCorrectionAdj = .05; // constant to scale down pivot correction angle to work with setting powers for mecanum drive motors
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        if (desiredRobotOrientation<getIntegratedHeading()-allowableAngleError) { //correct heading too
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double robotMovmentXComponent = calculateX(robotMovementAngle - globalPositionUpdate.returnOrientation(), robotPower);
            double robotMovmentYComponent = calculateY(robotMovementAngle - globalPositionUpdate.returnOrientation(), robotPower);
            double pivotCorrection = (desiredRobotOrientation - globalPositionUpdate.returnOrientation()) * pivotCorrectionAdj;
            //double[] powers = {robotMovmentYComponent, robotMovmentYComponent, robotMovmentYComponent, robotMovmentYComponent};//array for powers
            blPower = robotMovmentYComponent - robotMovmentXComponent + pivotCorrection;
            flPower = robotMovmentYComponent + robotMovmentXComponent + pivotCorrection;
            brPower = robotMovmentYComponent + robotMovmentXComponent - pivotCorrection;
            frPower = robotMovmentYComponent - robotMovmentXComponent - pivotCorrection;
            //set powers to motors to move
            double maxMotorPower = Math.max(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.abs(blPower)), Math.abs(brPower));

            if (Math.abs(maxMotorPower) > 1) {
                flPower = (flPower / maxMotorPower) * robotPower;
                frPower = (frPower / maxMotorPower) * robotPower;
                blPower = (blPower / maxMotorPower) * robotPower;
                brPower = (brPower / maxMotorPower) * robotPower;
            } else if (Math.abs(maxMotorPower) < .03) {
                flPower = 0;
                frPower = 0;
                blPower = 0;
                brPower = 0;
            }
            flMotor.setPower(flPower);
            frMotor.setPower(frPower);
            blMotor.setPower(blPower);
            brMotor.setPower(brPower);
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("XComponent: ", robotMovmentXComponent / .9);
            telemetry.addData("YComponent: ", robotMovmentYComponent / .9);
            telemetry.addData("Pivot Correction: ", pivotCorrection);
            telemetry.addData("Gyro orientation: ", imu.getAngularOrientation().firstAngle);
            telemetry.update();
            isGoToPosition = true;
            return false;
        }
        else{
            flMotor.setPower(0);
            frMotor.setPower(0);
            blMotor.setPower(0);
            brMotor.setPower(0);
            isGoToPosition = false;
            return true;

        }
    }
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    @Override
    public void stop() {
        if(globalPositionUpdate!=null) {
            globalPositionUpdate.stop();
        }
    }

    private double getIntegratedHeading() {
        if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) > 200) {
            rotations++;
        }
        else if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) < -200) {
            rotations--;
        }

        return (rotations * 360 + gyroAngles.firstAngle);
    }
    public void grip(boolean p)
    {
        if (p)
        {
            wobbleArmGripL.setPosition(1);
            wobbleArmGripR.setPosition(1);
        }
        else
        {
            wobbleArmGripL.setPosition(0);
            wobbleArmGripR.setPosition(0);
        }

    }
    public void hinge(boolean p)
    {
        if (p)// bring down arm
        {
            wobbleArmHingeL.setPower(-1);
            wobbleArmHingeR.setPower(1);

        }
        else {//bring arm back up
            wobbleArmHingeL.setPower(1); // open arm
            wobbleArmHingeR.setPower(-1); // open arm
             // continue for a second

        }
    }
    public void launch(){
        launcherL.setVelocity(-1000);
        launcherR.setVelocity(3000);//4250 accurate; too strong
        islaunchRunning = true;
    }
    public void launchSetZero(){
        launcherL.setVelocity(0);
        launcherR.setVelocity(0);
        islaunchRunning = false;
    }
     public void moveCollectorWheel()
     { //place after go to position statements to shoot at power sh
         collectorWheel.setTargetPosition(collectorWheel.getCurrentPosition() - (int)(3.5*CPICollectorWheel)); // enter encoder counts or inches you want to move times counts per inch FOR THIS WHEEL AND MOTORS
         collectorWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         collectorWheel.setPower(-1);
     }

}
