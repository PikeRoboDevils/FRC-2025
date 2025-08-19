package frc.robot.Utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Wrist.Wrist;

public class ElevatorWrist {

    private Elevator elevator;
    private Wrist wrist;

    private DoubleSupplier LeftY, RightY;
    
    private double leftSens = 2; 
    private double rightSens = 10; 

    public ElevatorWrist(Elevator elevator, Wrist wrist, DoubleSupplier LeftY, DoubleSupplier RightY) {
        this.elevator = elevator;
        this.wrist = wrist;
        this.LeftY = LeftY;
        this.RightY = RightY;
    }

// Doubled up for optional timeouts

    public Command stow() { 
        return Commands.parallel(elevator.setPoint(() -> 0),
                wrist.setAngle(() -> 30));
            }
    public Command stow(double timeout) { 
        return Commands.parallel(elevator.setPoint(() -> 0),
                wrist.setAngle(() -> 30))
                .withTimeout(timeout);
            }

    public Command coralSource(){
        return Commands.parallel(
            elevator.setPoint(() -> 6.19  - LeftY.getAsDouble()*leftSens),
            wrist.setAngle(() -> 26));
    }
    public Command coralSource(double timeout){
        return Commands.parallel(
            elevator.setPoint(() -> 6.19  - LeftY.getAsDouble()*leftSens),
            wrist.setAngle(() -> 26))
            .withTimeout(timeout);
    }

    public Command coralL1() {
        return Commands.parallel(
            elevator.setPoint(() -> 3 - LeftY.getAsDouble()* leftSens),
            wrist.setAngle(() -> 26 + RightY.getAsDouble()* rightSens));
    }
    public Command coralL1(double timeout) {
        return Commands.parallel(
            elevator.setPoint(() -> 3 - LeftY.getAsDouble()* leftSens),
            wrist.setAngle(() -> 26 + RightY.getAsDouble()* rightSens))
            .withTimeout(timeout);
    }

    public Command coralL2() {
        return Commands.parallel(
            elevator.setPoint(() -> 6.0 - LeftY.getAsDouble()* leftSens),
            wrist.setAngle(() -> 0 + RightY.getAsDouble()* rightSens));
    }
    public Command coralL2(double timeout) {
        return Commands.parallel(
            elevator.setPoint(() -> 6.0 - LeftY.getAsDouble()* leftSens),
            wrist.setAngle(() -> 0 + RightY.getAsDouble()* rightSens))
            .withTimeout(timeout);
    }


    public Command algaeL2() {
        return Commands.parallel(
            elevator.setPoint(() -> 3.4 - LeftY.getAsDouble()* leftSens),
            wrist.setAngle(() -> 30 + RightY.getAsDouble()* rightSens));
    }


    public Command coralL3() {
        return Commands.parallel(
            elevator.setPoint(() -> 11.76 - LeftY.getAsDouble() * leftSens), 
            wrist.setAngle(() -> 0 + RightY.getAsDouble()* rightSens));
    }
    public Command coralL3(double timeout) {
        return Commands.parallel(
            elevator.setPoint(() -> 11.76 - LeftY.getAsDouble() * leftSens), 
            wrist.setAngle(() -> 0 + RightY.getAsDouble()* rightSens))
            .withTimeout(timeout);
    }


    public Command algaeL3() {
        return Commands.parallel(
            elevator.setPoint(() -> 13.45 - LeftY.getAsDouble()* leftSens),
            wrist.setAngle(() -> 26 + RightY.getAsDouble()* rightSens));
    }
            

    public Command coralL4() {
        return Commands.parallel(
            elevator.setPoint(() -> 25.4 - LeftY.getAsDouble()* leftSens),            
            wrist.setAngle(() -> -50.65+ RightY.getAsDouble()* rightSens));
    }
    public Command coralL4(double timeout) {
        return Commands.parallel(
            elevator.setPoint(() -> 25.4 - LeftY.getAsDouble()* leftSens),            
            wrist.setAngle(() -> -50.65+ RightY.getAsDouble()* rightSens))
            .withTimeout(timeout);
    }

}