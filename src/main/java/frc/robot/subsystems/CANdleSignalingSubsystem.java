package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SignalingConstants;
import frc.utils.RGB;
import org.littletonrobotics.junction.Logger;

public class CANdleSignalingSubsystem extends SubsystemBase {

  private final CANdle candle = new CANdle(SignalingConstants.kCANdleID);

  public enum LightState {
    AUTON_IDLE,
    AUTON_FINISH,
    CORAL_MODE,
    CORAL_ACTION,
    ALGAE_MODE,
    ALGAE_ACTION,
    REEF,
    DISABLED
  };

  public LightState currentState = LightState.DISABLED;

  public Command setState(LightState newState) {
    currentState = newState;
    switch (newState) {
      case AUTON_IDLE:
        return setLights(0);
      case AUTON_FINISH:
        return setLights(1);
      case CORAL_MODE:
        return setLights(2);
      case CORAL_ACTION:
        return setLights(3);
      case ALGAE_MODE:
        return setLights(4);
      case ALGAE_ACTION:
        return setLights(5);
      case REEF:
        return setLights(6);
      default:
        return Commands.none();
    }
  }

  private Command setLights(int id) {
    switch (id) {
      case 0:
        return new InstantCommand(() -> setFullStrip(RGB.RED, false), this);
      case 1:
        return new InstantCommand(() -> setFullStrip(RGB.YELLOW, false));
      case 2:
        return new InstantCommand(() -> setFullStrip(RGB.BLUE, false));
      case 3:
        return new InstantCommand(() -> setFullStrip(RGB.BLUE, true));
      case 4:
        return new InstantCommand(() -> setFullStrip(RGB.GREEN, false));
      case 5:
        return new InstantCommand(() -> setFullStrip(RGB.GREEN, true));
      case 6:
        return new InstantCommand(() -> setFullStrip(RGB.PURPLE, false));
      case 7:
        return new InstantCommand(
            () ->
                setAnimation(
                    new TwinkleAnimation(
                        RGB.HOWDY_BLUE.red,
                        RGB.HOWDY_BLUE.blue,
                        RGB.HOWDY_BLUE.green,
                        0,
                        0.5,
                        SignalingConstants.NUMBER_OF_LEDS,
                        TwinklePercent.Percent42)));
      default:
        return Commands.none();
    }
  }

  private void setFullStrip(final RGB rgb, boolean flash) {
    clearAnimation();
    if (flash) {
      setAnimation(
          new StrobeAnimation(
              rgb.red, rgb.green, rgb.blue, 0, 0.5, SignalingConstants.NUMBER_OF_LEDS));
    } else {
      setSection(rgb, 8, SignalingConstants.NUMBER_OF_LEDS);
    }
  }

  private void setAnimation(Animation animation) {
    candle.animate(animation, 0);
  }

  public void clearAnimation() {
    candle.clearAnimation(0);
  }

  private void setSection(final RGB rgb, final int startID, final int count) {
    candle.setLEDs(rgb.red, rgb.green, rgb.blue, 0, startID, count);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Signaling/State", currentState.toString());
  }
}
