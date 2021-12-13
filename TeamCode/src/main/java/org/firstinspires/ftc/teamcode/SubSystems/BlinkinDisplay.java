package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.TestingOpModes.SampleRevBlinkinLedDriver;

/**
 * Definition of Subsystem Class <BR>
 *
 * Example : Intake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     <emsp>SUBSYSTEM1_SERVO_LEVEL1 for one state - example if intake motor is running, stopped, or reversing </emsp> <BR>
 *     <emsp>SUBSYSTEM1_SERVO_LEVEL2 for another state  = example if the intake is on or off </emsp> <BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     <emsp>runSubsystem1Motor checks if the motor is not running and runs the intake </emsp> <BR>
 *     <emsp>stopSubsystem1Motor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets subsystem1MotorState to SUBSYSTEM1_SERVO_LEVEL1.STOPPED </emsp> <BR>
 *     <emsp> startReverseSubsystem1Motor checks if the motor is not reversing, and sets the  motor to FORWARD, then also
 *     sets intake motor state to REVERSING</emsp> <BR>
 */
public class BlinkinDisplay {

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern patternDemo = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
    public RevBlinkinLedDriver.BlinkinPattern patternElementLoaded = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    public RevBlinkinLedDriver.BlinkinPattern patternEndGame = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER;
    public RevBlinkinLedDriver.BlinkinPattern patternBlack = RevBlinkinLedDriver.BlinkinPattern.BLACK;

    public BlinkinDisplay(HardwareMap hardwareMap){
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(patternDemo);
    }

    public void setPatternDemo(){
        blinkinLedDriver.setPattern(patternDemo);
    }

    public void setPatternElementLoaded(){
        blinkinLedDriver.setPattern(patternElementLoaded);
    }

    public void setPatternEndGame(){
        blinkinLedDriver.setPattern(patternEndGame);
    }

    public void setPatternBlack(){
        blinkinLedDriver.setPattern(patternBlack);
    }



}