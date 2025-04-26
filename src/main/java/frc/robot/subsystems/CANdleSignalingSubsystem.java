package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SignalingConstants;
import frc.utils.RGB;
import java.util.*;
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
    return runOnce(() -> currentState = newState);
  }

  private Command setLights(int id) {
    switch (id) {
      case 0:
        return runOnce(() -> setFullStrip(RGB.WHITE, false));
      case 1:
        return runOnce(() -> setFullStrip(RGB.YELLOW, false));
      case 2:
        return runOnce(() -> setFullStrip(RGB.BLUE, false));
      case 3:
        return runOnce(() -> setFullStrip(RGB.BLUE, true));
      case 4:
        return runOnce(() -> setFullStrip(RGB.GREEN, false));
      case 5:
        return runOnce(() -> setFullStrip(RGB.GREEN, true));
      case 6:
        return runOnce(() -> setFullStrip(RGB.PURPLE, false));
      case 7:
        return runOnce(() -> setDisabledAnimation());
      default:
        return Commands.none();
    }
  }

  private Command setDisabledAnimation() {
    return runOnce(
        () -> {
          setAnimation(AnimationType.TWINKLE, getAllianceColor(), 1.0);
        });
  }

  public Command setRandomAnimation() {
    if (currentState == LightState.DISABLED) {
      return runOnce(
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
    if (flash) {
      setAnimation(AnimationType.STROBE, rgb, 0.5);
    } else {
      setSection(rgb, 0, SignalingConstants.NUMBER_OF_LEDS);
    }
  }

  private void setAnimation(AnimationType animation, RGB rgb, double speed) {
    clearAnimation();
    switch (animation) {
      case FLOW:
        candle.animate(
            new ColorFlowAnimation(
                rgb.red,
                rgb.green,
                rgb.blue,
                0,
                speed,
                SignalingConstants.NUMBER_OF_LEDS,
                Direction.Forward));
        break;
      case FIRE:
        candle.animate(new FireAnimation(speed, 0.5, SignalingConstants.NUMBER_OF_LEDS, 0.7, 0.3));
        break;
      case LARSON:
        candle.animate(
            new LarsonAnimation(
                rgb.red,
                rgb.green,
                rgb.blue,
                0,
                speed,
                SignalingConstants.NUMBER_OF_LEDS,
                BounceMode.Front,
                12));
        break;
      case RAINBOW:
        candle.animate(new RainbowAnimation(1, speed, SignalingConstants.NUMBER_OF_LEDS));
        break;
      case RGB_FADE:
        candle.animate(new RgbFadeAnimation(1, speed, SignalingConstants.NUMBER_OF_LEDS));
        break;
      case FADE:
        candle.animate(
            new SingleFadeAnimation(
                rgb.red, rgb.green, rgb.blue, 0, speed, SignalingConstants.NUMBER_OF_LEDS));
        break;
      case STROBE:
        candle.animate(
            new StrobeAnimation(
                rgb.red, rgb.green, rgb.blue, 0, speed, SignalingConstants.NUMBER_OF_LEDS));
        break;
      case TWINKLE:
        candle.animate(
            new TwinkleAnimation(
                rgb.red,
                rgb.green,
                rgb.blue,
                0,
                speed,
                SignalingConstants.NUMBER_OF_LEDS,
                TwinklePercent.Percent42));
        break;
      case TWINKLE_OFF:
        candle.animate(
            new TwinkleOffAnimation(
                rgb.red,
                rgb.green,
                rgb.blue,
                0,
                speed,
                SignalingConstants.NUMBER_OF_LEDS,
                TwinkleOffPercent.Percent42));
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
    Logger.recordOutput(
        "Signaling/Hex", String.format("#%02x%02x%02x", rgb.red, rgb.green, rgb.blue));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Signaling/State", currentState.toString());
    switch (currentState) {
      case AUTON_IDLE:
        setFullStrip(RGB.WHITE, false);
      case AUTON_FINISH:
        setFullStrip(RGB.YELLOW, false);
      case CORAL_MODE:
        setFullStrip(RGB.BLUE, false);
      case CORAL_ACTION:
        setFullStrip(RGB.BLUE, true);
      case ALGAE_MODE:
        setLights(4).schedule();
      case ALGAE_ACTION:
        setLights(5).schedule();
      case REEF:
        setLights(6).schedule();
      case DISABLED:
        setFullStrip(getAllianceColor(), true);
    }
  }
}
