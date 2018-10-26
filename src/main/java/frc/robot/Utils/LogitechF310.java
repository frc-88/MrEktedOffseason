import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;

public class LogitechF310 {
Joystick controller;

public LogitechF310(int port){
    controller=new Joystick(port);
}

public double getLeftXAxis(){
    return controller.getRawAxis(0);
}

public double getLeftYAxis(){
    return -controller.getRawAxis(1);
}

public double getRightXAxis(){
    return controller.getRawAxis(4);
}

public double getRightYAxis(){
    return controller.getRawAxis(5);
}
public double getLeftTriggerAxis(){
    return controller.getRawAxis(2);
}
public double getRightTriggerAxis(){
    return controller.getRawAxis(3);
}
    
public boolean isLeftTriggerPressed(){
    return getLeftTriggerAxis()>.5;
}
public boolean isRightTriggerPressed(){
    return getRightTriggerAxis()>.5;

}
public Trigger getLeftTriggerTrigger(){
    return new Trigger(){

            @Override
            public boolean get() {
                return isLeftTriggerPressed();
            }
           

    };
}
public Trigger getRightTriggerTrigger(){
    return new Trigger(){

            @Override
            public boolean get() {
                return isRightTriggerPressed();
            }
           

    };
}

public boolean isAPressed(){
    return controller.getRawButton(0);
}
public Trigger getATrigger(){
    return new JoystickButton(controller, 0);
}

public boolean isBPressed(){
    return controller.getRawButton(1);
}
public Trigger getBTrigger(){
    return new JoystickButton(controller, 1);
}
public boolean isXPressed(){
    return controller.getRawButton(2);
}
public Trigger getXTrigger(){
    return new JoystickButton(controller, 2);
}
public boolean isYPressed(){
    return controller.getRawButton(3);
}
public Trigger getYTrigger(){
    return new JoystickButton(controller, 3);
}
public boolean isLBPressed(){
    return controller.getRawButton(4);
}
public Trigger getLBTrigger(){
    return new JoystickButton(controller, 4);
}
public boolean isRBPressed(){
    return controller.getRawButton(5);
}
public Trigger getRBTrigger(){
    return new JoystickButton(controller, 5);
}
public boolean isBackPressed(){
    return controller.getRawButton(6);
}
public Trigger getBackTrigger(){
    return new JoystickButton(controller, 6);
}
public boolean isStartPressed(){
    return controller.getRawButton(7);
}
public Trigger getStartTrigger(){
    return new JoystickButton(controller, 7);
}
public boolean isLeftStickPressed(){
    return controller.getRawButton(8);
}
public Trigger getLeftStickTrigger(){
    return new JoystickButton(controller, 8);
}
public boolean isRightStickPressed(){
    return controller.getRawButton(010);
}
public Trigger getRightStickTrigger(){
    return new JoystickButton(controller, 10);
}




}