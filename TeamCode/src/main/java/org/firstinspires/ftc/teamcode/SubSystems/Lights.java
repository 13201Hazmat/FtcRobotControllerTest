package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
/**
 * Definition of Subsystem Class <BR>
 *
 * Example : Intake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     INTAKE_SERVO_LEVEL1 for one state - example if intake motor is running, stopped, or reversing  <BR>
 *     INTAKE_SERVO_LEVEL2 for another state  = example if the intake is on or off  <BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     runIntakeMotor checks if the motor is not running and runs the intake  <BR>
 *     stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets intakeMotorState to INTAKE_SERVO_LEVEL1.STOPPED  <BR>
 *      startReverseIntakeMotor checks if the motor is not reversing, and sets the  motor to FORWARD, then also
 *     sets intake motor state to REVERSING <BR>
 */

public class Lights {

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern currentPattern;
    public RevBlinkinLedDriver.BlinkinPattern patternDemo = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
    public RevBlinkinLedDriver.BlinkinPattern patternIntakeOpenGrip = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    public RevBlinkinLedDriver.BlinkinPattern patternIntakeCloseGrip = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    public RevBlinkinLedDriver.BlinkinPattern patternTransferProgress = RevBlinkinLedDriver.BlinkinPattern.RED;
    public RevBlinkinLedDriver.BlinkinPattern patternOuttakeJunctionAligned = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    public RevBlinkinLedDriver.BlinkinPattern patternEndGame = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
    public RevBlinkinLedDriver.BlinkinPattern patternDefault = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
    public RevBlinkinLedDriver.BlinkinPattern patternBlack = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    public RevBlinkinLedDriver.BlinkinPattern patternWhite = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    public RevBlinkinLedDriver.BlinkinPattern patternRed = RevBlinkinLedDriver.BlinkinPattern.RED;
    public RevBlinkinLedDriver.BlinkinPattern patternYellow = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    public RevBlinkinLedDriver.BlinkinPattern patternGreen = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;

    public enum REV_BLINKIN_PATTERN {
        DEMO(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE),
        NONE(RevBlinkinLedDriver.BlinkinPattern.BLACK),
        TRANSFER_PROGRESS(RevBlinkinLedDriver.BlinkinPattern.RED),
        OUTTAKE_JUNCTION_NOT_ALIGNED(RevBlinkinLedDriver.BlinkinPattern.YELLOW),
        OUTTAKE_JUNCTION_ALIGNED(RevBlinkinLedDriver.BlinkinPattern.GREEN),
        END_GAME(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);

        private RevBlinkinLedDriver.BlinkinPattern blinkinPattern;
        private REV_BLINKIN_PATTERN(RevBlinkinLedDriver.BlinkinPattern blinkinPattern) {
            this.blinkinPattern = blinkinPattern;
        };
    }
    REV_BLINKIN_PATTERN revBlinkinPattern;

    public Lights(HardwareMap hardwareMap){
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(patternDemo);
        currentPattern = patternDemo;
    }

    public void setPattern(REV_BLINKIN_PATTERN revBlinkinPattern) {
        blinkinLedDriver.setPattern(revBlinkinPattern.blinkinPattern);
    }

    public void setPatternDemo(){
        if (currentPattern != patternDemo) {
            blinkinLedDriver.setPattern(patternDemo);
            currentPattern = patternDemo;
        }
    }

    public void setPatternEndGame(){
        if (currentPattern != patternEndGame) {
            blinkinLedDriver.setPattern(patternEndGame);
            currentPattern = patternEndGame;
        }
    }

    public void setPatternDefault(){
        if (currentPattern != patternDefault) {
            blinkinLedDriver.setPattern(patternDefault);
            currentPattern = patternDefault;
        }
    }

    public void setPatternBlack(){
        if (currentPattern != patternBlack) {
            blinkinLedDriver.setPattern(patternBlack);
            currentPattern = patternBlack;
        }
    }

    public void setPatternWhite(){
        if (currentPattern != patternWhite) {
            blinkinLedDriver.setPattern(patternWhite);
            currentPattern = patternWhite;
        }
    }

    public void setPatternRed(){
        if (currentPattern != patternRed) {
            blinkinLedDriver.setPattern(patternRed);
            currentPattern = patternRed;
        }
    }

    public void setPatternYellow(){
        if (currentPattern != patternYellow) {
            blinkinLedDriver.setPattern(patternYellow);
            currentPattern = patternYellow;
        }
    }

    public void setPatternGreen(){
        if (currentPattern != patternGreen) {
            blinkinLedDriver.setPattern(patternGreen);
            currentPattern = patternGreen;
        }
    }
}
