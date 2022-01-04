package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public RevBlinkinLedDriver.BlinkinPattern currentPattern;
    public RevBlinkinLedDriver.BlinkinPattern patternDemo = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
    public RevBlinkinLedDriver.BlinkinPattern patternElementLoaded = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    public RevBlinkinLedDriver.BlinkinPattern patternEndGame = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
    public RevBlinkinLedDriver.BlinkinPattern patternDefault = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
    public RevBlinkinLedDriver.BlinkinPattern patternBlack = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    public RevBlinkinLedDriver.BlinkinPattern patternWhite = RevBlinkinLedDriver.BlinkinPattern.WHITE;

    public BlinkinDisplay(HardwareMap hardwareMap){
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(patternDemo);
        currentPattern = patternDemo;
    }

    public void setPatternDemo(){
        blinkinLedDriver.setPattern(patternDemo);
        currentPattern = patternDemo;
    }

    public void setPatternElementLoaded(){
        blinkinLedDriver.setPattern(patternElementLoaded);
        currentPattern = patternElementLoaded;
    }

    public void setPatternEndGame(){
        blinkinLedDriver.setPattern(patternEndGame);
        currentPattern = patternEndGame;
    }

    public void setPatternDefault(){
        blinkinLedDriver.setPattern(patternDefault);
        currentPattern = patternDefault;
    }

    public void setPatternBlack(){
        blinkinLedDriver.setPattern(patternBlack);
        currentPattern = patternBlack;
    }

    public void setPatternWhite(){
        blinkinLedDriver.setPattern(patternWhite);
        currentPattern = patternWhite;
    }


}