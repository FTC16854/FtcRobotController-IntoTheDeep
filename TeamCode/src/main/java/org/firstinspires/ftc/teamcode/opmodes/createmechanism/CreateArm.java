package org.firstinspires.ftc.teamcode.opmodes.createmechanism;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.arm.NudgeArm;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmLevel;
import org.firstinspires.ftc.teamcode.commands.intake.MoveIntake;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

import java.util.HashMap;
import java.util.Map;

public class CreateArm {

    private ArmSubsystem arm;
    private final HardwareMap hwMap;
    private final String deviceName;
    private final Telemetry telemetry;
    private final GamepadEx op;

    private static final int NUDGE = 5;

    public CreateArm(final HardwareMap hwMap, final String deviceName, final GamepadEx op, Telemetry telemetry){
        this.deviceName = deviceName;
        this.hwMap = hwMap;
        this.op = op;
        this.telemetry = telemetry;

    }

    public CreateArm(final HardwareMap hwMap, final String deviceName, final GamepadEx op, Telemetry telemetry, boolean autoCreate){
        this.deviceName = deviceName;
        this.hwMap = hwMap;
        this.op = op;
        this.telemetry = telemetry;

        if (autoCreate) create();

    }

    public void create(){

        Map<Integer, Integer> armLevels = new HashMap<>();
        armLevels.put(0,0);
        armLevels.put(1,250);
        armLevels.put(2,600);
        armLevels.put(3,900);

        arm = new ArmSubsystem(hwMap,deviceName, DcMotorEx.RunMode.STOP_AND_RESET_ENCODER, (HashMap) armLevels, telemetry);

        arm.setArmTargetPosition(arm.getLevel(0));
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        NudgeArm nudgeArmUp = new NudgeArm(arm,NUDGE,telemetry);
        NudgeArm nudgeArmDown = new NudgeArm(arm, -NUDGE, telemetry);

        SetArmLevel moveToLevel0 = new SetArmLevel(arm,0, telemetry);
        SetArmLevel moveToLevel1 = new SetArmLevel(arm,1, telemetry);
        SetArmLevel moveToLevel2 = new SetArmLevel(arm,2, telemetry);
        SetArmLevel moveToLevel3 = new SetArmLevel(arm,3, telemetry);

        Button armNudgerUp = new GamepadButton(op, GamepadKeys.Button.DPAD_UP);
        Button armNudgerDown = new GamepadButton(op, GamepadKeys.Button.DPAD_DOWN);

        //Button armNudgerUp = new GamepadButton(toolOp2, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        //Button armNudgerDown = new GamepadButton(toolOp2, GamepadKeys.Button.RIGHT_STICK_BUTTON);

        armNudgerUp.whileHeld(nudgeArmUp);
        armNudgerDown.whileHeld(nudgeArmDown);

        //A Level 0
        Button armLevel0 = new GamepadButton(op, GamepadKeys.Button.A);
        //X Level 1
        Button armLevel1 = new GamepadButton(op, GamepadKeys.Button.X);
        //Y Level 2
        Button armLevel2 = new GamepadButton(op, GamepadKeys.Button.Y);
        //B Level 3
        Button armLevel3 = new GamepadButton(op, GamepadKeys.Button.B);

        armLevel0.whenPressed(moveToLevel0);
        armLevel1.whenPressed(moveToLevel1);
        armLevel2.whenPressed(moveToLevel2);
        armLevel3.whenPressed(moveToLevel3);


    }

    public ArmSubsystem getArm(){
        return arm;
    }
}
