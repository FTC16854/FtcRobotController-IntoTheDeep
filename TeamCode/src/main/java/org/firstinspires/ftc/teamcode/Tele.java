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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "TeleOp", group = "Robot")
public class Tele extends OpMode {
    Hardware robot = new Hardware();
    double speedLimit = 1;
    double oldTime;
    int autoState = 1;
    int liftPos = 0;
    double rotationSpeedLimit;
    double dpadPressTime = 0;
    private ElapsedTime runtime = new ElapsedTime();
    boolean currentGripSwitch;
    boolean lastGripSwitch = false;

    enum gunnerControlMode {
        LUKE,
        NORMAL,
        ONEMAN;
    }

    gunnerControlMode gunner;
    double wrist_UP = 0.1;
    double wrist_MID = 0.625;
    double wrist_DOWN = 1;

    double grip_OPEN = 0;
    double grip_CLOSED = 0.37;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors

        // Send telemetry message to signify robot waiting;
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        robot.imu.initialize(parameters);
        robot.imu.startAccelerationIntegration(new Position(DistanceUnit.MM, 0, 0, 0, 0), new Velocity(DistanceUnit.MM, 0, 0, 0, 0), 5);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.upperLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.upperLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.upperLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.gripper.setPosition(0.5);
        robot.wrist.setPosition(0.1);
        gunner = gunnerControlMode.NORMAL;
        telemetry.setMsTransmissionInterval(20);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Robot Ready", "");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speedLimit);


        if (gamepad2.start && gamepad2.x) {
            gunner = gunnerControlMode.NORMAL;
        } else if (gamepad2.start && gamepad2.y) {
            gunner = gunnerControlMode.LUKE;
        } else if (gamepad1.start && gamepad1.x) {
            gunner = gunnerControlMode.ONEMAN;
        }

        if (gamepad1.right_trigger > 0.5) {
            speedLimit = 0.3;
            rotationSpeedLimit = 0.35;
        } else if (robot.lift.getCurrentPosition() > 300) {
            speedLimit = 0.5674;
            rotationSpeedLimit = 0.5;
        } else {
            speedLimit = 0.9;
            rotationSpeedLimit = 0.9;
        }

        /** Josh Gunner Controls **/
        if (gunner == gunnerControlMode.NORMAL) {
//            if (robot.toggleSwitch.getVoltage() < 3) {
//                if (gamepad2.left_stick_y < 0) { //UP
//                    if (robot.upperUpSwitch.getVoltage() < 3) {
//                        robot.upperLift.setPower(0);
//                    } else {
//                        if (robot.upperLift.getCurrentPosition() < 2200 || robot.upperLift.getCurrentPosition() > 600) {
//                            robot.upperLift.setPower(-gamepad2.left_stick_y);
//                        } else {
//                            robot.upperLift.setPower(-gamepad2.left_stick_y * .3);
//                        }
//                        robot.lift.setPower(0);
//                    }
//                } else { //DOWN
//                    if (robot.upperDownSwitch.getVoltage() < 3) {
//                        robot.upperLift.setPower(-gamepad2.left_stick_y);
//                        robot.lift.setPower(0);
//                    } else {
//                        robot.upperLift.setPower(0);
//                        robot.lift.setPower(-gamepad2.left_stick_y);
//                    }
//                }
//            } else if (robot.lift.getCurrentPosition() < 3000) {
//                robot.lift.setPower(-gamepad2.left_stick_y);
//            } else {
//                robot.lift.setPower(-gamepad2.left_stick_y * .65);
//            }
            if (robot.toggleSwitch.getVoltage() < 3) {
                if (gamepad2.left_stick_y < 0) {
                    robot.lift.setPower(0);
                } else{
                    robot.lift.setPower(-gamepad2.left_stick_y);
                }
            } else if (robot.bottomSwitch.getVoltage() > 3) {
                if (gamepad2.left_stick_y > 0) {
                    robot.lift.setPower(0);
                    robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                } else {
                    robot.lift.setPower(-gamepad2.left_stick_y);
                }
            } else {
                robot.lift.setPower(-gamepad2.left_stick_y);
            }

            if (robot.upperUpSwitch.getVoltage() < 3) {
                if (gamepad2.right_stick_y < 0) {
                    robot.upperLift.setPower(0);
                } else{
                    robot.upperLift.setPower(-gamepad2.right_stick_y);
                }
            } else if (robot.upperDownSwitch.getVoltage() > 3) {
                if (gamepad2.right_stick_y > 0) {
                    robot.upperLift.setPower(0);
                    robot.upperLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                } else {
                    robot.upperLift.setPower(-gamepad2.right_stick_y);
                }
            } else {
                robot.upperLift.setPower(-gamepad2.right_stick_y);
            }

            if (gamepad2.left_stick_y > .05 || gamepad2.left_stick_y < -.05 || gamepad2.right_stick_y > .05 || gamepad2.right_stick_y < -.05) {
                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.upperLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftPos = 0;
            } else if (gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right) {
                robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.upperLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


            if (gamepad2.dpad_up) {
                liftPos = 4;
            } else if (gamepad2.dpad_left) {
                liftPos = 3;
            } else if (gamepad2.dpad_down) {
                liftPos = 2;
            } else if (gamepad2.dpad_right) {
                robot.upperLift.setTargetPosition(0);
                robot.upperLift.setPower(1);
                robot.lift.setTargetPosition(0);
                robot.lift.setPower(1);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.upperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            switch (liftPos) {
                case 0:
                    break;
                case 1:
                    liftPos = 2;
                    break;
                case 2: //Low junction
                    robot.upperLift.setTargetPosition(2940);
                    robot.upperLift.setPower(.85);
                    robot.lift.setTargetPosition(690);
                    robot.lift.setPower(.85);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.upperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;
                case 3: //Medium junction
                    robot.upperLift.setTargetPosition(2940);
                    robot.upperLift.setPower(.85);
                    robot.lift.setTargetPosition(2080);
                    robot.lift.setPower(.85);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.upperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;
                case 4: //High junction
                    robot.upperLift.setTargetPosition(2940);
                    robot.upperLift.setPower(.85);
                    robot.lift.setTargetPosition(4920);
                    robot.lift.setPower(.85);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.upperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;
                case 5:
                    liftPos = 4;
                    break;
            }

            if (gamepad2.x) {
                robot.wrist.setPosition(wrist_MID);
            } else if (gamepad2.y || gamepad1.y) {
                robot.wrist.setPosition(wrist_UP);
            } else if (gamepad2.a) {
                robot.wrist.setPosition(wrist_DOWN);
            }

            switch (autoState) {
                case 0: //Do not run
                    break;
                case 1:
                    if (gamepad2.b) {
                        oldTime = runtime.milliseconds();
                        autoState++;
                        robot.gripper.setPosition(grip_OPEN);
                    } else if (gamepad2.left_trigger > 0.5) {
                        robot.gripper.setPosition(grip_OPEN);
                        robot.gripperWheel.setPower(0);
                    } else {
                        robot.gripper.setPosition(grip_CLOSED);
                    }
                    break;
                case 2:
                    if (runtime.milliseconds() > oldTime + 200) {
                        robot.wrist.setPosition(wrist_UP);
                        autoState = 1;
                    }
                    break;
            }

            switch (autoState) {
                case 0: //Do not run
                    break;
                case 1:
                    if (gamepad2.b) {
                        oldTime = runtime.milliseconds();
                        autoState++;
                        robot.gripper.setPosition(0);
                    }
                    break;
                case 2:
                    if (runtime.milliseconds() > oldTime + 200) {
                        robot.wrist.setPosition(0.1);
                        autoState = 1;
                    }
                    break;
            }

            if (robot.gripSwitch.getVoltage() < 3) {
                currentGripSwitch = true;
            } else {
                currentGripSwitch = false;
            }

            if (gamepad2.right_trigger > 0.5 && robot.gripSwitch.getVoltage() > 3) {
                robot.gripperWheel.setPower(1);
                currentGripSwitch = true;
            } else {
                robot.gripperWheel.setPower(-0.05);

                if (currentGripSwitch && !lastGripSwitch) {
                    if (!gamepad2.isRumbling())  // Check for possible overlap of rumbles.
                        gamepad2.rumble(.55,.55,150);
                        gamepad1.rumble(.55,.55,150);
                }
                lastGripSwitch = currentGripSwitch;
            }

            /** One Man Gunner Controls **/
        } else if (gunner == gunnerControlMode.ONEMAN) {

        }

        telemetry.addData("Lift Encoder", robot.lift.getCurrentPosition());
        telemetry.addData("Upper Lift Encoder", robot.upperLift.getCurrentPosition());
        telemetry.addData("Lift Position", liftPos);
        telemetry.addData("Gunner Control Mode:", gunner);
        telemetry.addData("Upper Top", robot.upperUpSwitch.getVoltage());
        telemetry.addData("Upper Bottom", robot.upperDownSwitch.getVoltage());
        

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public double getHeading() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        if (heading < -180) {
            heading = heading + 360;
        } else if (heading > 180) {
            heading = heading - 360;
        }
        return heading;
    }

    public void drive(double y_axis, double x_axis, double rotation, double speedLimit) {
        double stick_x = x_axis;
        double stick_y = y_axis;
        double theta = 0;
        double joyR = 0;

        //MOVEMENT
        rotation = rotation * rotationSpeedLimit;
        theta = Math.atan2(stick_y, stick_x);
        joyR = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2));

        robot.frontRightDrive.setPower(((Math.sin((Math.PI / 4) - theta) * joyR) + rotation) * speedLimit);
        robot.frontLeftDrive.setPower(((Math.sin(((3 * Math.PI) / 4) - theta) * joyR) + rotation) * speedLimit);
        robot.backLeftDrive.setPower(((Math.sin(((5 * Math.PI) / 4) - theta) * joyR) + rotation) * speedLimit);
        robot.backRightDrive.setPower(((Math.sin(((7 * Math.PI) / 4) - theta) * joyR) + rotation) * speedLimit);

    }
}
