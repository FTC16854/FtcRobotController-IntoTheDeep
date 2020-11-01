package org.firstinspires.ftc.teamcode.Qualifier_1.Accesories;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;


public class WobbleGoal {

    DcMotor wobbleGoalMotor;


    private Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    public WobbleGoal(DcMotor inputWobbleGoalMotor){

        wobbleGoalMotor = inputWobbleGoalMotor;
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleGoalMotor.setPower(0.5);
    }

    public void position0() {

        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setTargetPosition(0);

    }

    public void position1() {
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setTargetPosition(1200);
    }

    public void position2() {

        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setTargetPosition(700);


    }

    public void position3() {

        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setTargetPosition(1000);


    }
}

