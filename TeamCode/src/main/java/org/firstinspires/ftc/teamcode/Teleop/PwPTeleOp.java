package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

@TeleOp
public class PwPTeleOp extends LinearOpMode {
    PwPRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new PwPRobot(this,true);
//        robot.cv.observeSleeve();
//        Pose2d startPose = new Pose2d(-29.6, 62.25, toRadians(90));
//        robot.roadrun.setPoseEstimate(startPose);//        robot.cv.observeStick();
        robot.cv.observeCone();
        waitForStart();
//        robot.cv.observeStick();
        resetRuntime();
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(!isStopRequested()){
            logger.loopcounter++;
            robot.teleOp();
            telemetry.update();
        }
        robot.stop();
    }
}