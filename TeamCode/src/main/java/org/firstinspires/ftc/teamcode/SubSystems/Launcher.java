package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher {
    public Servo launcherServo;

    public enum LAUNCHER_STATE{
        LAUNCHER_PULLED_BACK(0), //UPDATE
        LAUNCHER_LAUNCHED(0); //UPDATE

        private double launcherPosition;

        LAUNCHER_STATE(double moveLauncherPosition){
            this.launcherPosition = moveLauncherPosition;
        }

        public double getLauncherPosition(){return launcherPosition;}
    }
    public LAUNCHER_STATE launcherState = LAUNCHER_STATE.LAUNCHER_PULLED_BACK;

    public Telemetry telemetry;
    public Launcher(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        launcherServo = hardwareMap.get(Servo.class, "launcher");
        initLauncher();
    }

    public void initLauncher(){
        launcherServo.setPosition(LAUNCHER_STATE.LAUNCHER_PULLED_BACK.getLauncherPosition());
        launcherState = LAUNCHER_STATE.LAUNCHER_PULLED_BACK;
    }

    public void launchDrone(){
        launcherServo.setPosition(LAUNCHER_STATE.LAUNCHER_LAUNCHED.getLauncherPosition());
        launcherState = LAUNCHER_STATE.LAUNCHER_LAUNCHED;
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("=============");
    }
}
