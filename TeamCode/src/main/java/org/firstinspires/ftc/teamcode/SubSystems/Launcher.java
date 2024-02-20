package org.firstinspires.ftc.teamcode.SubSystems;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher {
    public Servo launcherServo;

    public enum LAUNCHER_SERVO_STATE {
        LAUNCHER_PULLED_BACK(0.26), //TODO : Update Value
        LAUNCHER_LAUNCHED(0.5); //TODO : Update Value

        private double launcherPosition;

        LAUNCHER_SERVO_STATE(double moveLauncherPosition){
            this.launcherPosition = moveLauncherPosition;
        }

        public double getLauncherPosition(){return launcherPosition;}
    }
    public LAUNCHER_SERVO_STATE launcherServoState = LAUNCHER_SERVO_STATE.LAUNCHER_PULLED_BACK;

    public enum LAUNCHER_BUTTON_STATE{
        SAFE,
        ARMED,
        LAUNCHED
    }
    public LAUNCHER_BUTTON_STATE launcherButtonState = LAUNCHER_BUTTON_STATE.SAFE;
    public ElapsedTime launcherClickTimer = new ElapsedTime(MILLISECONDS);
    public double LAUNCHER_BUTTON_ARMED_THRESHOLD = 700;

    public Telemetry telemetry;
    public Launcher(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        launcherServo = hardwareMap.get(Servo.class, "launcher_servo");
        initLauncher();
    }

    public void initLauncher(){
        launcherServo.setPosition(LAUNCHER_SERVO_STATE.LAUNCHER_PULLED_BACK.getLauncherPosition());
        launcherServoState = LAUNCHER_SERVO_STATE.LAUNCHER_PULLED_BACK;
        launcherButtonState = LAUNCHER_BUTTON_STATE.SAFE;
    }

    public void launchDrone(){
        launcherServo.setPosition(LAUNCHER_SERVO_STATE.LAUNCHER_LAUNCHED.getLauncherPosition());
        launcherServoState = LAUNCHER_SERVO_STATE.LAUNCHER_LAUNCHED;
    }

    public void printDebugMessages(){
        //******  debug ******
        telemetry.addLine("Launcher");
        telemetry.addData("    Button State", launcherButtonState);
        telemetry.addData("    Servo state", launcherServoState);
        telemetry.addData("    Servo position", launcherServo.getPosition());
        telemetry.addLine("=============");
    }
}
