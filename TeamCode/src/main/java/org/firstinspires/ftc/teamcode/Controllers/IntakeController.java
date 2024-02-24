package org.firstinspires.ftc.teamcode.Controllers;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Magazine;

public class IntakeController {
    public Intake intake;
    public Magazine magazine;
    LinearOpMode currentOpMode;


    public IntakeController(Intake intake, Magazine magazine, LinearOpMode currentOpMode){
        this.intake = intake;
        this.magazine = magazine;
        this.currentOpMode = currentOpMode;
    }


    public void dropLiftIntakeAfterYellowDrop(){
        intake.moveIntakeLiftDown();
    }

    public Action dropLiftIntake(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                dropLiftIntakeAfterYellowDrop();
                return false;
            }
        };
    }

    public void squishPurplePixelInStartOfAutoForDrop(){
        squishHorizPurplePixel();
        safeWaitMilliSeconds(70);
        intake.stopHorizIntakeInward();
    }

    public Action squishPurplePixelInStartOfAutoForDropAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                squishPurplePixelInStartOfAutoForDrop();
                return false;
            }
        };
    }

    public void dropPurplePixelUsingIntake(){
        reverseIntakeForPurplePixelDrop();
        //intake.reverseIntake();
        safeWaitMilliSeconds(200);//200
        intake.stopHorizIntakeReverse();
    }

    public Action dropPurplePixelUsingIntakeAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                dropPurplePixelUsingIntake();
                return false;
            }
        };
    }

    public void dropPurplePixelUsingIntakeWithWait(double waitTimeSeconds){
        safeWaitMilliSeconds(waitTimeSeconds*1000);
        reverseIntakeForPurplePixelDrop();
        //intake.reverseIntake();
        safeWaitMilliSeconds(200);//200
        intake.stopHorizIntakeReverse();
    }

    public Action dropPurplePixelUsingIntakeWithWaitAction(double waitTimeSeconds){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                dropPurplePixelUsingIntakeWithWait(waitTimeSeconds);
                return false;
            }
        };
    }


    public void intakeAtStackTwoPixels(){
        intake.moveIntakeLiftDown();
        intake.startIntakeInward();
        safeWaitMilliSeconds(150);
        //safeWaitMilliSeconds(150);
        intake.startIntakeHorizToCollect(); //TODO: RUN BASED ON MAGAZINE
        //magazine.senseMagazineState();
        safeWaitMilliSeconds(1400);//2600
        intake.reverseIntake();
        intake.reverseIntakeHoriz();
        safeWaitMilliSeconds(500); //700
        intake.stopHorizIntakeInward();
        safeWaitMilliSeconds(100);
        intake.stopIntake();
    }

    public void intakeAtStackOnePixel(){
        intake.moveIntakeLiftDown();
        intake.startIntakeInward();
        safeWaitMilliSeconds(150);
        //safeWaitMilliSeconds(150);
        intake.startIntakeHorizToCollect(); //TODO: RUN BASED ON MAGAZINE
        safeWaitMilliSeconds(650);
        intake.reverseIntake();
        intake.reverseIntakeHoriz();
        safeWaitMilliSeconds(250);
        intake.stopHorizIntakeInward();
        safeWaitMilliSeconds(200);
        intake.stopIntake();
    }

    public void intakeLiftUp(){
        intake.moveRollerHeight(Intake.INTAKE_ROLLER_HEIGHT.LIFTED);
    }

    public Action intakeLiftUpAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                intakeLiftUp();
                return false;
            }
        };
    }

    public Action intakeAtStackOnePixelAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                intakeAtStackOnePixel();
                return false;
            }
        };
    }

    public Action intakeAtStackTwoPixelsAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                intakeAtStackTwoPixels();
                return false;
            }
        };
    }

    public void reverseIntakeForPurplePixelDrop() {
        if(intake.intakeMotorState != Intake.INTAKE_MOTOR_STATE.REVERSING) {
            intake.moveRollerHeight(Intake.INTAKE_ROLLER_HEIGHT.DROPPED);
            safeWaitMilliSeconds(170);
            intake.reverseIntakeHoriz();
        }
    }

    public void squishHorizPurplePixel() { //TODO : WHAT IS THIS?
        if(intake.intakeMotorState != Intake.INTAKE_MOTOR_STATE.REVERSING) {
            //intake.moveRollerHeight(Intake.INTAKE_ROLLER_HEIGHT.DROPPED);
            intake.startIntakeHorizToCollect();
        }
    }

    public void reverseIntakeLiftAfterYellowDrop(){
        if(intake.intakeMotorState != Intake.INTAKE_MOTOR_STATE.REVERSING) {
            intake.reverseIntakeHoriz();
        }
    }

    public void safeWaitMilliSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!currentOpMode.isStopRequested() && timer.time() < time) {
        }
    }
}
