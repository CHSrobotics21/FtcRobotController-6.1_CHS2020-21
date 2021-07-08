
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp(name = "teleOphwMapextend")
//@Disabled
public class TeleOpSimplifiedOpModeExtend extends HardwareMapClassOpModeExtend{
    /*
    Simplified tele op class that calls hardwareMapClass that has goToPosition, other methods, and a new hardware map class
    This class also has a method called correct odometry where when the robot is driven into a corner and you press a
    button on the dpad (gamepad1), it attempts to reset the position of the robot to the correct coordinates of those
    corners so that the robot can line up in the right positions more accurately
    4/2/21
     */
    HardwareMapClass robo= new HardwareMapClass();
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS), timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    double x, y, joystickAngle, joystickAngle360,  fieldReference = 0, frPower=0, flPower=0, brPower=0, blPower=0, maxMotorPower=0, driveSpeed = 0, driveRotation = 0;
    int rings = 0;
    boolean robotPerspective = false, ringStopperToggle = false, ringStopperCanToggle = true, topPrevious = false, topCurrent = false, isWheelRunning = false, lockDrive = false, launchToggle = false, gripToggle = false, collectorToggle = false, collectorCanToggle = true, collectorReverseCanToggle = true, collectorReverseToggle = true, launcherCanToggle = true, gripCanToggle = true;
    public Double startX, startY, startOrientation;

    @Override
    public void init() {

        super.initHwMap(hardwareMap);
        File TeleOpStartingPos = AppUtil.getInstance().getSettingsFile("TeleOpStartingPos.txt");
        String fileContents = ReadWriteFile.readFile(TeleOpStartingPos).trim();
        String[] array = fileContents.split(" ");
        startX = Double.parseDouble(array[0]);
        startY = Double.parseDouble(array[1]);
        startOrientation = Double.parseDouble(array[2]);
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75, startX, startY, startOrientation);
        positionThread = new Thread(globalPositionUpdate);

        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseLeftEncoder();
        ringStopper.setPosition(0);

        telemetry.addData("Status", "Initialized");

        telemetry.addData("StartingPostionX", startX);
        telemetry.addData("StartingPostionY", startY);
        telemetry.addData("StartingOrientation", startOrientation);
        telemetry.update();
    }


    @Override
    public void init_loop() {
        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("StartingPostionX", startX);
        telemetry.addData("StartingPostionY", startY);
        telemetry.addData("StartingOrientation", startOrientation);
        telemetry.update();
    }


    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {
        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        isWheelRunning = collectorWheel.getPower() != 0;

        if(gamepad1.dpad_up){
            correctOdometry(54, 128);//remote upper left corner
        }
        else if(gamepad1.dpad_right){
            correctOdometry(128, 128);//remote upper right corner
        }
        else if(gamepad1.dpad_left)
        {
            correctOdometry(54, 8.5);//remote lower left corner
        }
        else if(gamepad1.dpad_down){
            correctOdometry(128,8.5);//remote lower right corner
        }

        if(gamepad2.left_bumper){
            collectorWheelRun(-1); //run collector wheel(with power; method runs set mode run without encoders) towards launcher
        }
        else if(gamepad2.left_trigger>.05){
            collectorWheelRun(1); //reverse collector wheel run
        }
        else if(intakeDistanceSensor.getDistance(DistanceUnit.INCH)<6.5&&intakeDistanceSensor.getDistance(DistanceUnit.INCH)>0 &&rings < 3){
            collectorWheelRun(-1); //run collector wheel when there is intake of rings through the collector
            timer.reset();
        }
        else{
            collectorWheelRun(0);
        }

        if(timer.time()>2&&timer.time()<2.05){
            rings++;
            collectorWheelRun(0);
        }
        topPrevious = topCurrent;
        topCurrent = ringStopperSensor.getDistance(DistanceUnit.CM)<4.6&&ringStopperSensor.getDistance(DistanceUnit.CM)>0;
        if (topPrevious && !topCurrent){
            rings--;
        }
        if (ringStopperSensor.getDistance(DistanceUnit.CM)<4.6&&ringStopperSensor.getDistance(DistanceUnit.CM)>0 && islaunchRunning){
            moveCollectorWheel();
        }
        changeLauncherAngle(.45,gamepad2.dpad_up );//highest launcher angle
        changeLauncherAngle(.4,gamepad2.dpad_left );//middle launcher angle
        changeLauncherAngle(.3,gamepad2.dpad_down );//lowest launcher angle

        controlToggle(ringStopperToggle, ringStopperCanToggle, ringStopper, gamepad2.dpad_right, false, false);//use ringStopper
        controlToggle(collectorToggle, collectorCanToggle, collector, -.9, gamepad2.right_trigger);//run collector
        controlToggle(collectorReverseToggle, collectorReverseCanToggle, collector, .9, gamepad2.right_bumper);//run collector reversed
        controlToggle(launchToggle, launcherCanToggle, ringStopper, gamepad2.x, true, false);//launch toggle
        controlToggle(gripToggle, gripCanToggle, wobbleArmGripR, gamepad2.b, false, true);// gripper toggle

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

        lockDrive = islaunchRunning && isWheelRunning;

        if (gamepad1.right_trigger > .05 || gamepad1.left_trigger > .05) {
            driveRotation = gamepad1.left_trigger - gamepad1.right_trigger;
            desiredRobotHeading = getIntegratedHeading();
        } else if (Math.abs(desiredRobotHeading - getIntegratedHeading()) > 5) {
            driveRotation = (desiredRobotHeading - getIntegratedHeading()) / (Math.abs(desiredRobotHeading - getIntegratedHeading())) * .05;
        }
        else {
            driveRotation = 0;
        }
        if (gamepad1.dpad_up) {robotPerspective = true;}
        if (gamepad1.dpad_down) {robotPerspective = false;}
        if (robotPerspective) { //Controls are mapped to the robot perspective
            fieldReference = 0;
            //Positive values for x axis are joystick right
            //Positive values for y axis are joystick down
            y = Range.clip(-gamepad1.left_stick_y,-1,1);
            x = Range.clip(-gamepad1.left_stick_x,-1,1);
        } else {   //Controls are mapped to the field
            fieldReference = desiredRobotHeading;
            //Positive values for x axis are joystick right
            //Positive values for y axis are joystick down
            y = Range.clip(gamepad1.left_stick_x,-1,1);
            x = Range.clip(-gamepad1.left_stick_y,-1,1);
        }
        joystickAngle = Math.atan2(x,y);
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
            if(gamepad1.x){ //change powershot positions to what the best way to score them is like angled or just one position where you would change the angle for each powershot
                goToPositionSlowDown(89, 67, .7, 0, 2);//powershot 1
            }
            else if(gamepad1.a){
                goToPositionSlowDown(82, 67, .7, 0, 2); //powershot 2
            }
            else if(gamepad1.b){
                goToPositionSlowDown(75, 67, .7, 0, 2); //powershot 3
            }
            else if(gamepad1.y){
                goToPositionSlowDown(105, 39, .7, 0, 2);// launching high tower goal
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
            driveTrainZero();
        }

        telemetry.addData("rings: ",rings);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Wobble counts", brMotor.getCurrentPosition());
        telemetry.addData("StartingPostionX", globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);
        telemetry.addData("StartingPostionY", globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH);
        telemetry.addData("intake distance Sensor: ", String.format("%.01f cm",intakeDistanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("sensor timer: ", timer.time());
        telemetry.addData("previous: ", topPrevious);
        telemetry.addData("current", topCurrent);
        telemetry.addData("Distance ring stopper: ", ringStopperSensor.getDistance(DistanceUnit.CM));
    }


    @Override
    public void stop() {
        globalPositionUpdate.stop();
    }
// class methods below


    public void controlToggle(boolean toggle, boolean canToggle, DcMotor motor, double onPower, boolean gamepadControl){
        if(gamepadControl){
            if(canToggle) //make sure that the code doesn't just toggle the thing every iteration as long as the trigger's held
            {
                canToggle=false;
                if(toggle)
                {//if the motor is currently running, turn it off
                    motor.setPower(0); //turn off the motor
                    toggle=false; //remember that the collector motor has been turned off
                }
                else
                {//if the motor isn't currently running, turn it on
                    motor.setPower(onPower); //turn on motor
                    toggle=true; //remember that the motor has been turned on
                }
            }
        }
        else
        {
            canToggle=true;
        }
    }
    public void controlToggle(boolean toggle, boolean canToggle, DcMotor motor, double onPower, float gamepadControl){
        if(gamepadControl > .05)
        {
            if(canToggle) //make sure that the code doesn't just toggle the thing every iteration as long as the button is held
            {
                canToggle=false;

                if(toggle) //if motor is currently running, turn it off:
                {
                    motor.setPower(0); //turn off the launcher motor
                    toggle=false; //remember that the launcher motor has been turned off
                }
                //if the launcher isn't currently running, run this code to turn it on:
                else
                {
                    motor.setPower(onPower); //turn on the launcher motor
                    toggle =true; //remember that the launcher motor has been turned on
                }
            }
        }
        else
        {
            canToggle=true;
        }
    }
    public void controlToggle(boolean toggle, boolean canToggle, Servo servo, boolean gamepadControl, boolean isDoLaunch, boolean isDoGrip ){
        if(gamepadControl){
            if(canToggle) //make sure that the code doesn't just toggle the thing every iteration as long as the trigger's held
            {
                canToggle=false;
                if(toggle)
                {//if the servo was previously in the 1 position, go 0
                    if(isDoLaunch){
                        launchSetZero();
                    }
                    if(isDoGrip){
                        grip(false);
                    }
                    else {
                        servo.setPosition(0); //servo go 0 unless it is in the situation where you want to grip
                    }
                    toggle=false; //remember that the servo has gone 0
                }
                else
                {//if the servo is 0, servo 1
                    if(isDoLaunch){
                        launch();
                    }
                    if(isDoGrip){
                        grip(true);
                    }
                    else {
                        servo.setPosition(1); //turn on the collector motor
                    }
                    toggle=true; //remember that the collector motor has been turned on
                }
            }
        }
        else
        {
            canToggle=true;
        }
    }
    public void changeLauncherAngle(double servoAngle, boolean gamepadControl){
        if (gamepadControl){
            launcherAngleR.setPosition(servoAngle);
            launcherAngle.setPosition(servoAngle);
        }
    }
    public void correctOdometry(double cornerX, double cornerY){
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75, cornerX, cornerY, 0);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
    }
}
