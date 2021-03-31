package org.firstinspires.ftc.teamcode;/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@TeleOp(name = "TeleOnBot", group = "Example")
@Disabled
public class IterativeTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    DcMotor frMotor, flMotor, brMotor, blMotor, collectorWheel, collector;
    DcMotorEx launcherR, launcherL;
    Servo launcherAngle, launcherAngleR;
    BNO055IMU imu;
    Orientation gyroAngles;
    ColorSensor colorSensor;
    Servo wobbleArmGripL, wobbleArmGripR;
    CRServo wobbleArmHingeL, wobbleArmHingeR;
    DcMotor verticalLeft, verticalRight, horizontal;
    DistanceSensor intakeDistanceSensor, outDistanceSensor;

    double frPower=0, flPower=0, brPower=0, blPower=0, collectorPower, launchPower;
    double maxMotorPower=0;
    double driveSpeed = 0;
    double driveRotation = 0;
    double a, b, x, y, joystickAngle, joystickAngle360;
    double desiredRobotHeading;
    int rotations = 0;
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

    @Override
    public void init() {
        frMotor = hardwareMap.dcMotor.get("frontright");
        flMotor = hardwareMap.dcMotor.get("frontleft");
        brMotor = hardwareMap.dcMotor.get("backright");
        blMotor = hardwareMap.dcMotor.get("backleft");
        collector = hardwareMap.dcMotor.get("collector");

        launcherR = hardwareMap.get(DcMotorEx.class, "launcherR");
        launcherL = hardwareMap.get(DcMotorEx.class, "launcherL");
        collectorWheel = hardwareMap.dcMotor.get("wheel");
        launcherAngle = hardwareMap.get(Servo.class, "ServoL");
       launcherAngleR = hardwareMap.get(Servo.class, "ServoR");
        wobbleArmGripL = hardwareMap.servo.get("GripL");
        wobbleArmGripR = hardwareMap.servo.get("GripR");
        wobbleArmHingeL = hardwareMap.crservo.get("HingeL");
        wobbleArmHingeR = hardwareMap.crservo.get("HingeR");
        intakeDistanceSensor = hardwareMap.get(DistanceSensor.class, "intake");
        outDistanceSensor = hardwareMap.get(DistanceSensor.class, "out");


        verticalLeft = hardwareMap.dcMotor.get("backleft");
        verticalRight = hardwareMap.dcMotor.get("frontleft");
        horizontal = hardwareMap.dcMotor.get("frontright");
        //brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Status", "Initialized");
        File TeleOpStartingPos = AppUtil.getInstance().getSettingsFile("TeleOpStartingPos.txt");
        String fileContents = ReadWriteFile.readFile(TeleOpStartingPos).trim();
        String[] array = fileContents.split(" ");
        Double startX = Double.parseDouble(array[0]);
        Double startY = Double.parseDouble(array[1]);
        Double startOrientation = Double.parseDouble(array[2]);

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75, startX, startY, startOrientation);
        telemetry.addData("StartingPostionX", startX);
        telemetry.addData("StartingPostionY", startY);
        telemetry.addData("StartingOrientation", startOrientation);
        telemetry.update();
        positionThread = new Thread(globalPositionUpdate);
    }


    @Override
    public void init_loop() {
        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("1", "Integrated Heading: " + getIntegratedHeading());
        telemetry.addData("2", "heading: " + globalPositionUpdate.returnOrientation());
        telemetry.addData("StartingPostionX", globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);
        telemetry.addData("StartingPostionY", globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH);
        // telemetry.addData("1 Right Motor Pos", frMotor.getCurrentPosition());
        // telemetry.addData("2 Left Motor Pos", flMotor.getCurrentPosition());

        telemetry.addData("intake distance Sensor: ", String.format("%.01f cm",intakeDistanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("out distance Sensor: ", String.format("%.01f cm",outDistanceSensor.getDistance(DistanceUnit.CM)));
    }


    @Override
    public void start() {
        runtime.reset();

        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseLeftEncoder();
    }


    @Override
    public void loop() {
        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
        telemetry.update();

        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        y = Range.clip(gamepad1.right_stick_x, -1, 1);
        x = Range.clip(gamepad1.right_stick_y, -1, 1);
        joystickAngle = Math.atan2(-x, y);
        joystickAngle360 = joystickAngle >= 0 ? joystickAngle : (2 * Math.PI) + joystickAngle;

        driveSpeed = Range.clip(Math.sqrt(y * y + x * x), -1, 1);
        if (gamepad1.right_trigger > .05 || gamepad1.left_trigger > .05) {
            driveRotation = gamepad1.left_trigger - gamepad1.right_trigger;
            desiredRobotHeading = getIntegratedHeading();
        } else if (Math.abs(desiredRobotHeading - getIntegratedHeading()) > 5) {
            driveRotation = (desiredRobotHeading - getIntegratedHeading()) / (Math.abs(desiredRobotHeading - getIntegratedHeading())) * .05;
        } else {
            driveRotation = 0;
        }


        if (gamepad1.a)
       {
            launcherAngleR.setPosition(.5);
            launcherAngle.setPosition(.5);
            launcherL.setPower(1);
            launcherR.setPower(-1);
           if (goToPosition(105, 82, .9, 0, 1))
           {
            collector.setPower(.9);
            collectorWheel.setPower(.9);
           }
           hinge(false);//close hinge
       }
       else{
        launcherL.setPower(0);
        launcherR.setPower(0);
        collector.setPower(0);
        collectorWheel.setPower(0);
       }

//       if (gamepad1.y)
//       {
//           //change the gripper to closed
//           //change the hinge to closed
//           grip(false);
//           hinge(false);
//           if (goToPosition(globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH, 8, .9, 90, .2))
//           {
//               hinge(true); //lower hinge
//               grip(true);//release grip
//           }
//           hinge(false);//close hinge
//       }
//         if(gamepad2.y){
//             launcherL.setPower(1);
//             launcherR.setPower(1);
//         }
//         else{
//             launcherL.setPower(0);
//             launcherR.setPower(0);
//         }

        if(gamepad1.y) {
            launcherL.setPower(.85);
            launcherR.setPower(-.85);

            if (!buttonPressed) {
                timer.reset();
                buttonPressed = true;
            } else if (timer.time() > .2) {
                collectorWheel.setPower(-.9);
            }
        }
        else if(gamepad1.x) {
            launcherL.setPower(.85);
            launcherR.setPower(-.85);
        }
        else{
            buttonPressed=false;
            launcherL.setPower(0);
            launcherR.setPower(0);
            AIlaunch = true;//collectorWheel.setPower(0);
        }

        if(gamepad2.left_bumper){
            collectorWheel.setPower(-.9);
        }
        else if(gamepad2.left_trigger > .05){
            collectorWheel.setPower(.9);
        }
        else if(intakeDistanceSensor.getDistance(DistanceUnit.INCH)<6.5||intakeDistanceSensor.getDistance(DistanceUnit.INCH)>20&&!stopSensor){
            collectorWheel.setPower(-.9);
        }
        else if(AIlaunch){
            collectorWheel.setPower(0);
            AIlaunch=false;
        }
        else{

            collectorWheel.setPower(0);
        }

        if(gamepad1.left_bumper){
            stopSensor = true;
        }
        if(gamepad1.right_bumper){
            stopSensor = false;
        }// stop distance sensor movements to run manual

        if(gamepad2.right_bumper){
            collector.setPower(.7);
        }
        else{
            collector.setPower(0);
        }

        if (gamepad2.a){
            grip(true);
        }
        else if (gamepad2.b){
            grip(false);
        }

        if(gamepad2.y)
        {
            hinge(true);
        }
        else if(gamepad2.x) {
            hinge(false);
        }
        else{
            wobbleArmHingeR.setPower(0);
            wobbleArmHingeL.setPower(0);
        }

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
//        if (gamepad1.a) {
//            if (!goToPosition(107, 72, .9, 0, 1)) {
//                launcherAngleR.setPosition(.5);
//                launcherAngle.setPosition(.5);
//            } else {
//                launcherL.setPower(1);
//                launcherR.setPower(-1);
//                collector.setPower(.9);
//                collectorWheel.setPower(.9);
//            }
//        }
//        else {
//            collector.setPower(0);
//            collectorWheel.setPower(0);
//            launcherL.setPower(0);
//            launcherR.setPower(0);
//        }
//       if (gamepad1.b) {// for power shots so the robot just goes through it when we have three rings
//           launcherL.setPower(1);
//           launcherR.setPower(-1);
//           launcherAngleR.setPosition(.4);// might need to change the position for the power shots
//           launcherAngle.setPosition(.4);
//           if (goToPosition(81, 72, .9, 0, .2)) {
//               moveCollectorWheel();
//               if (goToPosition(74, 72, .9, 0, .2)) {
//                   moveCollectorWheel();
//                   if (goToPosition(74, 80, 0.9, 0, .2)) {
//                      moveCollectorWheel();
//                   }
//               }
//           }
//       }
//       else {
//               launcherL.setPower(0);
//               launcherR.setPower(0);
//           }
//
//            if(gamepad2.right_bumper) {
//            collector.setPower(.4);
//        }
//        else if(gamepad2.left_bumper){
//            collector.setPower(-.4);
//        }
//        else{
//            collector.setPower(0);
//        }
    //}


        if (gamepad1.b){

            launcherAngleR.setPosition(.4);
            launcherAngle.setPosition(.4);

        }
        if(gamepad1.a){

            launcherAngleR.setPosition(.5);
            launcherAngle.setPosition(.5);

        }

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Wobble counts", brMotor.getCurrentPosition());
    }



    public boolean goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError ) {
        targetXPosition *= COUNTS_PER_INCH;
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
//            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
//            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
//            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
//            telemetry.update();
//            telemetry.addData("XComponent: ", robotMovmentXComponent / .9);
//            telemetry.addData("YComponent: ", robotMovmentYComponent / .9);
//            telemetry.addData("Pivot Correction: ", pivotCorrection);

            return false;
        }
        else{
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
        globalPositionUpdate.stop();
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
    public void moveCollectorWheel()
    { //place after go to position statements to shoot at power shot

        collectorWheel.setTargetPosition(collectorWheel.getCurrentPosition() - (int)(5*CPICollectorWheel)); // enter encoder counts or inches you want to move times counts per inch FOR THIS WHEEL AND MOTORS
        collectorWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        collectorWheel.setPower(1);
        timer.reset();
        while(timer.time()<1){

        }
        collectorWheel.setPower(0);
    }
    public void launch(){
        launcherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherR.setVelocity(200);
    }
}

