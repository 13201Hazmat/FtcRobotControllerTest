package org.firstinspires.ftc.teamcode.Controllers;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

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
        intake.stopStackIntake();
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
        intake.stopStackIntake();
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
        intake.stopStackIntake();
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
        intake.startStackIntakeToCollect(); //TODO: RUN BASED ON MAGAZINE
        //magazine.senseMagazineState();
        safeWaitMilliSeconds(1400);//2600
        intake.reverseIntake();
        intake.reverseStackIntake();
        safeWaitMilliSeconds(500); //700
        intake.stopStackIntake();
        safeWaitMilliSeconds(100);
        intake.stopIntake();
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

    public void intakeAtStackOnePixel(){
        intake.moveIntakeLiftDown();
        intake.startIntakeInward();
        safeWaitMilliSeconds(150);
        //safeWaitMilliSeconds(150);
        intake.startStackIntakeToCollect(); //TODO: RUN BASED ON MAGAZINE
        safeWaitMilliSeconds(1000);
        intake.reverseIntake();
        intake.reverseStackIntake();
        safeWaitMilliSeconds(250);
        intake.stopStackIntake();
        safeWaitMilliSeconds(200);
        intake.stopIntake();
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

    public void intakeAtStackUsingMagazineSensor( double magazineTimeOutSeconds){
        ElapsedTime stackIntakeMagazineTimeOut = new ElapsedTime(SECONDS);
        intake.moveIntakeLiftDown();
        intake.startIntakeInward();
        safeWaitMilliSeconds(150);
        //safeWaitMilliSeconds(150);
        intake.startStackIntakeToCollect();
        stackIntakeMagazineTimeOut.reset();
        magazine.senseMagazineState();
        while (stackIntakeMagazineTimeOut.time()<magazineTimeOutSeconds &&
                magazine.magazineState != Magazine.MAGAZINE_STATE.LOADED_TWO_PIXEL) {
            magazine.senseMagazineState();
        }
    }

    //Alternate to run stack Intake
    public void intakeAtStackUsingMagazineSensor1( double magazineTimeOutSeconds){
        ElapsedTime stackIntakeMagazineTimeOut = new ElapsedTime(SECONDS);
        intake.moveIntakeLiftDown();
        intake.startIntakeInward();
        safeWaitMilliSeconds(100);//150
        //intake.startStackIntakeToCollect();
        stackIntakeMagazineTimeOut.reset();
        magazine.senseMagazineState();
        while (stackIntakeMagazineTimeOut.time() < magazineTimeOutSeconds &&
                magazine.magazineState != Magazine.MAGAZINE_STATE.LOADED_TWO_PIXEL) {
            intake.runStackIntakeOneRotationAuto();
            safeWaitMilliSeconds(290);//400
            magazine.senseMagazineState();
        }
    }


    public Action intakeAtStackUsingMagazineSensorAction(double magazineTimeOutSeconds){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                intakeAtStackUsingMagazineSensor1(magazineTimeOutSeconds);
                return false;
            }
        };
    }

    public void intakeReverse( double timeSeconds){
        ElapsedTime timer = new ElapsedTime(SECONDS);
        intake.reverseIntake();
        safeWaitMilliSeconds(timeSeconds*1000);
        intake.stopStackIntake();
        safeWaitMilliSeconds(200);
        intake.stopIntake();
    }

    public Action intakeReverseAction(double timeSeconds){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                intakeReverse(timeSeconds);
                return false;
            }
        };
    }



    public void intakeLiftUp(){
        intake.moveRollerHeight(Intake.STACK_INTAKE_LIFT_STATE.LIFTED);
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


    public void reverseIntakeForPurplePixelDrop() {
        if(intake.intakeMotorState != Intake.INTAKE_MOTOR_STATE.REVERSING) {
            intake.moveRollerHeight(Intake.STACK_INTAKE_LIFT_STATE.DROPPED);
            safeWaitMilliSeconds(170);
            intake.reverseStackIntake();
        }
    }


    public Action reverseStackIntakeOneRotationAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                intake.runReverseStackIntakeOneRotationAuto();
                return false;
            }
        };
    }

    public void squishHorizPurplePixel() { //TODO : WHAT IS THIS?
        if(intake.intakeMotorState != Intake.INTAKE_MOTOR_STATE.REVERSING) {
            //intake.moveRollerHeight(Intake.INTAKE_ROLLER_HEIGHT.DROPPED);
            intake.startStackIntakeToCollect();
        }
    }

    public void reverseIntakeLiftAfterYellowDrop(){
        if(intake.intakeMotorState != Intake.INTAKE_MOTOR_STATE.REVERSING) {
            intake.reverseStackIntake();
        }
    }

    public void safeWaitMilliSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!currentOpMode.isStopRequested() && timer.time() < time) {
        }
    }
}
