package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.SignalingConstants;
import frc.utils.RGB;
import org.littletonrobotics.junction.Logger;
import java.util.*;

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

  public enum AnimationType {
    FLOW,
    FIRE,
    LARSON,
    RAINBOW,
    RGB_FADE,
    FADE,
    STROBE,
    TWINKLE,
    TWINKLE_OFF
  }

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
    case DISABLED:
        return setLights(7);
      default:
        return Commands.none();
    }
  }

  private Command setLights(int id) {
    switch (id) {
      case 0:
        return new InstantCommand(() -> setFullStrip(RGB.WHITE, false));
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
                setDisabledAnimation()
        );
      default:
        return Commands.none();
    }
  }

  private Command setDisabledAnimation() {
    return new InstantCommand(
        () -> {
          setAnimation(AnimationType.TWINKLE, getAllianceColor(), 1.0);
    });
  }

  public Command setRandomAnimation() {
    if (currentState == LightState.DISABLED) {
      return new InstantCommand(
        () -> {
            AnimationType[] animations = AnimationType.values();
            AnimationType randomAnimation = animations[(int) (Math.random() * animations.length)];
            setAnimation(randomAnimation, getAllianceColor(), 1.0);
        });
    } else {
      return Commands.none();
    }
  }

  private RGB getAllianceColor() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return RGB.RED;
    } else {
      return RGB.HOWDY_BLUE;
    }
  }

  private void setFullStrip(final RGB rgb, boolean flash) {
    clearAnimation();
    if (flash) {
      setAnimation(AnimationType.STROBE, rgb, 0.5);
    } else {
      setSection(rgb, 8, SignalingConstants.NUMBER_OF_LEDS);
    }
  }

  private void setAnimation(AnimationType animation, RGB rgb, double speed) {
    clearAnimation();
    switch (animation) {
        case FLOW:
            candle.animate(new ColorFlowAnimation(rgb.red, rgb.green, rgb.blue, 0, speed, SignalingConstants.NUMBER_OF_LEDS, Direction.Forward));
            break;
        case FIRE:
            candle.animate(new FireAnimation(speed, 0.5, SignalingConstants.NUMBER_OF_LEDS, 0.7, 0.3));
            break;
        case LARSON:
            candle.animate(new LarsonAnimation(rgb.red, rgb.green, rgb.blue, 0, speed, SignalingConstants.NUMBER_OF_LEDS, BounceMode.Front, 12));
            break;
        case RAINBOW:
            candle.animate(new RainbowAnimation(1, speed, SignalingConstants.NUMBER_OF_LEDS));
            break;
        case RGB_FADE:
            candle.animate(new RgbFadeAnimation(1, speed, SignalingConstants.NUMBER_OF_LEDS));
            break;
        case FADE:
            candle.animate(new SingleFadeAnimation(rgb.red, rgb.green, rgb.blue, 0, speed, SignalingConstants.NUMBER_OF_LEDS));
            break;
        case STROBE:
            candle.animate(new StrobeAnimation(rgb.red, rgb.green, rgb.blue, 0, speed, SignalingConstants.NUMBER_OF_LEDS));
            break;
        case TWINKLE:
            candle.animate(new TwinkleAnimation(rgb.red, rgb.green, rgb.blue, 0, speed, SignalingConstants.NUMBER_OF_LEDS, TwinklePercent.Percent42));
            break;
        case TWINKLE_OFF:
            candle.animate(new TwinkleOffAnimation(rgb.red, rgb.green, rgb.blue, 0, speed, SignalingConstants.NUMBER_OF_LEDS, TwinkleOffPercent.Percent42));
            break;
        default:
            clearAnimation();
            break;
    }
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
