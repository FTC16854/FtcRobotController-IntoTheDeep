/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Original FTC opmode header block
 *
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 **/

/** Parent OpMode Class
 * All Teleop and Autonomous OpModes should inherit from (extend) ParentOpMode.
 * Each child/subclass OpMode should have its own unique runOpMode() method that will
 * override the ParentOpMode runOpMode() method.
 **/

@TeleOp(name="Parent Opmode Example", group="Linear Opmode")
@Disabled
public class ParentOpMode extends LinearOpMode {

    // Declare OpMode members, hardware variables
    public ElapsedTime runtime = new ElapsedTime();

    private DcMotorSimple rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotorSimple leftBack = null;
    private DcMotorSimple Intake = null;
    private DcMotor lift = null;
    private DcMotor extension = null;
    // Sensor Items

    private DigitalChannel bottomLimitSwitch = null;
    private DigitalChannel inwardsLimitSwitch = null;

    SparkFunOTOS OdometrySensor;

    //Other Global Variables
    SparkFunOTOS.Pose2D pos;

    //Lift Positions
    int MinHeightLimitForExtension = 15624;
    int liftBottom = 0;
    int lowBasket = 18000;
    int highBasket = 36000;
    int liftTop = 48000;
    int targetLiftPos = liftBottom;

    //Extension Positions
    int ExtensionLimitBelow = 17564;
    int ExtensionIn = 0;
    int ExtensionMid = 24000;
    int ExtensionOutLimit = 48000;
    int targetExtensionPos = ExtensionIn;

    public void initialize(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Driver Station app or Driver Hub).
        rightFront = hardwareMap.get(DcMotorSimple.class, "rf_drive");
        rightBack = hardwareMap.get(DcMotor.class, "rb_drive");
        leftFront = hardwareMap.get(DcMotor.class,"lf_drive");
        leftBack = hardwareMap.get(DcMotorSimple.class, "lb_drive");
        Intake = hardwareMap.get(DcMotorSimple.class, "Intake");
        lift = hardwareMap.get(DcMotor.class, "lift");
        extension = hardwareMap.get(DcMotor.class, "extension");

        //Sensors
        bottomLimitSwitch = hardwareMap.get(DigitalChannel.class, "bottom_limit");
        inwardsLimitSwitch = hardwareMap.get(DigitalChannel.class, "Inward_limit");

        OdometrySensor = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();

        //Set motor run mode (if using SPARK Mini motor controllers)


        //Set Motor  and servo Directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        // is not known if accurate
        extension.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set brake or coast modes
        // (NOTE: SPARK MINI MOTOR CONTROLLER SWITCHES MUST BE SET TO SAME AS DRIVE MOTOR CONFIGS)
        // BRAKE or FLOAT (Coast)
        //rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Update Driver Station Status Message after init
        telemetry.addData("Status:", "Initialized");
        telemetry.update();
    }

    /**
     * runOpMode() will be overridden in child OpMode.
     * Basic structure should remain intact (init, wait for start, while(opModeIsActive),
     * Additionally, Emergency conditions should be checked during every cycle
     */
    @Override
    public void runOpMode() {

        initialize();

        // Init loop - optional
        while(opModeInInit()){
            // Code in here will loop continuously until OpMode is started
        }

        // Wait for the game to start (driver presses PLAY) - May not be needed if using an init Loop
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            // code here should never actually execute in parent opmode.
            // This function will be be overridden by child opmode classes



            //include emergency stop check in all runOpMode() functions/methods
            //implementation depends on which E-stop function will be used (boolean/void)
            //checkEmergencyStop(); // Stops motors and Terminates if buttons are pressed
            //without additional code in the while(opModeIsActive) loop.

            telemetry.update();
        }
    }

    /*****************************/
    //Controls should be mapped here to avoid accessing gamepad directly in other functions/methods
    //This also makes it simpler to re-map controls as desired
    //CONTROLLER MAP

    // Thumbsticks
    public double left_sticky_x(){
        return gamepad1.left_stick_x;
    }
    public double left_sticky_y() { return -gamepad1.left_stick_y;}
    public double right_sticky_y() { return -gamepad1.right_stick_y;}
    public double right_sticky_x() { return  gamepad1.right_stick_x;}

    // Buttons
    public boolean emergencyButtons(){
        // check for combination of buttons to be pressed before returning true
        return (gamepad1.y && gamepad1.x) || (gamepad2.y && gamepad2.x);
    }
    public boolean buttonLiftUp() { return  gamepad1.right_bumper;}
    public boolean buttonLiftDown() { return gamepad1.left_bumper;}
    public boolean buttonLiftBottom() { return gamepad1.x;}
    public boolean buttonLiftHigh() { return gamepad1.y;}
    public boolean buttonLiftLow() { return gamepad1.a;}
    public double outtake_trigger() { return gamepad1.left_trigger;}
    public double intake_trigger() { return gamepad1.right_trigger;}
    public boolean buttonExtensionOut() { return gamepad2.y;}
    public boolean buttonExtensionMid() { return  gamepad2.a;}
    public boolean buttonExtensionIn() { return gamepad2.x;}
    public boolean buttonExtensionForward() { return gamepad2.left_bumper;}
    public boolean buttonExtensionBackward() { return gamepad2.right_bumper;}

    public boolean triggerButton(){
        if(gamepad1.right_trigger>.25){
            return true;         // Converts analog triggers into digital button presses (booleans)
        }
        else{
            return false;
        }
    }


    /****************************/
    // Emergency Stop Functions

    public void checkEmergencyStop(){
        if(emergencyButtons()){
            stopper();
            terminateOpModeNow();
        }
    }



    /*****************************/
    //Drive Methods

    // Assign left and right drive speed using arguments/parameters rather than hardcoding
    // thumb stick values inside function body. This will allow tank drive to be reused for
    // autonomous programs without additional work
    public void tankDrive(double left, double right){
        leftFront.setPower(left);
        leftBack.setPower(left);
        rightFront.setPower(right);
        rightBack.setPower(right);
    }

    public void holonomic(){
        double magnitude = Math.hypot(left_sticky_x(), left_sticky_y());
        double offset = Math.toRadians(-90);
        double angle = Math.atan2(left_sticky_y(), left_sticky_x())+offset;
        double rotateVelocity = right_sticky_x();

        double Vlf = (magnitude * Math.cos(angle +(Math.PI/4))+rotateVelocity);
        double Vlb = (magnitude * Math.sin(angle +(Math.PI/4))+rotateVelocity);
        double Vrf = (magnitude * Math.sin(angle +(Math.PI/4))-rotateVelocity);
        double Vrb = (magnitude * Math.cos(angle +(Math.PI/4))-rotateVelocity);

        leftFront.setPower(Vlf);
        leftBack.setPower(Vlb);
        rightFront.setPower(Vrf);
        rightBack.setPower(Vrb);

        telemetry.addData("lf", Vlf);
        telemetry.addData("lb", Vlb);
        telemetry.addData("rf", Vrf);
        telemetry.addData("rb", Vrb);
    }

    public void holonomicFieldCentric (){
        double magnitude = Math.hypot(left_sticky_x(), left_sticky_y());
        double robotHead = getAngler();
        double offset = Math.toRadians(-90+robotHead);
        double angle = Math.atan2(left_sticky_y(), left_sticky_x())+offset;
        double rotateVelocity = right_sticky_x();

        double Vlf = (magnitude * Math.cos(angle +(Math.PI/4))+rotateVelocity);
        double Vlb = (magnitude * Math.sin(angle +(Math.PI/4))+rotateVelocity);
        double Vrf = (magnitude * Math.sin(angle +(Math.PI/4))-rotateVelocity);
        double Vrb = (magnitude * Math.cos(angle +(Math.PI/4))-rotateVelocity);

        leftFront.setPower(Vlf);
        leftBack.setPower(Vlb);
        rightFront.setPower(Vrf);
        rightBack.setPower(Vrb);

        telemetry.addData("lf", Vlf);
        telemetry.addData("lb", Vlb);
        telemetry.addData("rf", Vrf);
        telemetry.addData("rb", Vrb);
        telemetry.addData("angle", robotHead);
    }


    public void stopper(){
        tankDrive(0,0);
    }

    /*****************************/
    //More Methods (Functions)
    public void Taker(){
        double inTaker = intake_trigger();
        double outTaker = -outtake_trigger();

        if (inTaker > 0.15) {
            Intake.setPower(inTaker);
        }
        else{
            if (outTaker > 0.15) {
                Intake.setPower(outTaker);
            }

            else {
                Intake.setPower(0);
            }
        }

    }


    /*****************************/
    //Autonomous Functions

    /*****************************/
    //Encoder Functions
   /*

    */

    /*****************************/
    //Gyro Functions

    //Sensor functions
    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        OdometrySensor.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        OdometrySensor.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        OdometrySensor.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        OdometrySensor.setLinearScalar(1.0);
        OdometrySensor.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        OdometrySensor.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        OdometrySensor.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        OdometrySensor.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        OdometrySensor.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    public void ZeroOtosSensor(){
        OdometrySensor.resetTracking();
    }

    public void CalibrateOtosSensor(){
        OdometrySensor.calibrateImu();
    }

    public void TrackingOtos(){
        pos = OdometrySensor.getPosition();

        double Xvalue;
        double Yvalue;
        double angle;

        Xvalue = pos.x;
        Yvalue = pos.y;
        angle = pos.h;


        telemetry.addData("xylophone",Xvalue);
        telemetry.addData("yttrium",Yvalue);
        telemetry.addData("protractor",angle);
    }

    public double getAngler() {
        double angle;

        pos = OdometrySensor.getPosition();

        angle = pos.h;
        return angle;
    }

    //place the lift functions below

    public int getLiftPosition(){
        return lift.getCurrentPosition();
    }

    public void setLift0(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean liftAtBottom(){
        return !bottomLimitSwitch.getState();
    }

    public void hominglift(){
        while(!liftAtBottom() && opModeIsActive()){
            lift.setPower(-0.3);
        }
        lift.setPower(0);
        setLift0();
    }

    public void goToPosLift(int Pos){
        double speed;
        speed =0.7;
        lift.setTargetPosition(Pos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(speed);
    }

    public void setLiftPos(){

        int smallMargin;

        smallMargin = 257;

        if(buttonLiftDown()){
            targetLiftPos = getLiftPosition() - smallMargin;
        }

        if(buttonLiftUp()){
            targetLiftPos = getLiftPosition() + smallMargin;
        }

        if(buttonLiftBottom()) {
            targetLiftPos = liftBottom;
        }

        if(buttonLiftHigh()) {
            targetLiftPos = highBasket;
        }

        if(buttonLiftLow()) {
            targetLiftPos = lowBasket;
        }


        if(targetLiftPos < liftBottom) {
            targetLiftPos = liftBottom;
        }

        if(targetLiftPos > liftTop) {
            targetLiftPos = liftTop;
        }

        goToPosLift(targetLiftPos);
    }



    public int getExtensionPosition(){
        return extension.getCurrentPosition();
    }

    public void setExtension0(){
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean extensionAtInside(){
        return inwardsLimitSwitch.getState();
    }

    public void homingExtension(){
        while(!extensionAtInside() && opModeIsActive()){
            extension.setPower(-0.3);
        }
        extension.setPower(0);
        setExtension0();
    }

    public void goToPosExtension(int Pos){
        double speed;
        speed =0.7;
        extension.setTargetPosition(Pos);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(speed);
    }

    public void setExtensionPos(){
        int smallMargin;
        int ExtensionOut;

        ExtensionOut=ExtensionOutLimit;
        smallMargin = 257;

        if (getLiftPosition()<MinHeightLimitForExtension){
            ExtensionOut = ExtensionLimitBelow;
        }


        if(buttonExtensionBackward()){
            targetExtensionPos = getExtensionPosition() - smallMargin;
        }

        if(buttonExtensionForward()){
            targetExtensionPos = getExtensionPosition() + smallMargin;
        }



        if(buttonExtensionIn()) {
            targetExtensionPos = ExtensionIn;
        }

        if (buttonExtensionMid()) {
            targetLiftPos = ExtensionMid;
        }

        if (buttonExtensionOut()) {
            targetLiftPos = ExtensionOut;
        }

        if(targetExtensionPos < ExtensionIn) {
            targetExtensionPos = ExtensionIn;
        }

        if(targetExtensionPos > ExtensionOut) {
            targetExtensionPos = ExtensionOut;
        }

        goToPosExtension(targetExtensionPos);
    }

    // autonomous functions below

    public void autoIntake() {
        Intake.setPower(0.7);
    }

    public void autoOuttake() {
        Intake.setPower(-0.5);
    }

    public void autoIntakeStop() {
        Intake.setPower(0);
    }

    public void autoHolonomicFieldCentric (double magnitude, double angle, double rotateVelocity){
        double robotHead = getAngler();
        double offset = Math.toRadians(-90+robotHead);
        angle = Math.toRadians(angle)+offset;

        double Vlf = (magnitude * Math.cos(angle +(Math.PI/4))+rotateVelocity);
        double Vlb = (magnitude * Math.sin(angle +(Math.PI/4))+rotateVelocity);
        double Vrf = (magnitude * Math.sin(angle +(Math.PI/4))-rotateVelocity);
        double Vrb = (magnitude * Math.cos(angle +(Math.PI/4))-rotateVelocity);

        leftFront.setPower(Vlf);
        leftBack.setPower(Vlb);
        rightFront.setPower(Vrf);
        rightBack.setPower(Vrb);

        telemetry.addData("lf", Vlf);
        telemetry.addData("lb", Vlb);
        telemetry.addData("rf", Vrf);
        telemetry.addData("rb", Vrb);
        telemetry.addData("angle", robotHead);
    }

    public void autorotate (double targetAngle, double speed) {
        double breakPoint = 3;
        while (true && opModeIsActive()) {
            if (getAngler()>targetAngle) {
                autoHolonomicFieldCentric(0,0,-speed);
            } else {
                autoHolonomicFieldCentric(0, 0, speed);
            }
            if (getAngler()<targetAngle+breakPoint && getAngler()>targetAngle-breakPoint) {
                stopper();
                break;
            }

            telemetry.update();
        }
    }

    public void timedAutoHolonomicFieldCentric (double magnitude, double angle, long milliTime){
        autoHolonomicFieldCentric(magnitude,angle,0);
        sleep (milliTime);
        stopper();
    }


    // Test Functions
    public void displayPositionTelemetry(){
        telemetry.addData("Lift at Bottom:",liftAtBottom());
        telemetry.addData("Extension Retracted:",extensionAtInside());
        telemetry.addData("Lift Position:",getLiftPosition());
        telemetry.addData("Extension Position:",getExtensionPosition());
    }

    public void manualLiftAndExtension(){
        double liftPower = 0.3;
        double extensionPower = 0.3;

        if(buttonLiftUp()){
            lift.setPower(liftPower);
        }
        else{
            if(buttonLiftDown()){
                lift.setPower((-liftPower));
            }
            else{
                lift.setPower(0);
            }
        }

        if(buttonExtensionForward()){
            extension.setPower(extensionPower);
        }
        else{
            if(buttonExtensionBackward()){
                extension.setPower(-extensionPower);
            }
            else{
                extension.setPower(0);
            }
        }

    }

}