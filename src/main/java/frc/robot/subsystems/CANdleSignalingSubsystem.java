package frc.robot.subsystems;

import frc.robot.Constants.SignalingConstants;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import frc.utils.RGB;



public class CANdleSignalingSubsystem extends SubsystemBase{

    private final CANdle candle = new CANdle(SignalingConstants.kCANdle);

    public enum LightState {
        IDLE,
        WAYPOINT,
        CORAL_MODE,
        CORAL_INTAKE,
        ALGAE_MODE,
        ALGAE_INTAKE,
        REEF
      };

      public LightState currentState = LightState.IDLE;

      private void setState(LightState newState) {
        currentState = newState;
        switch (newState) {
          case IDLE:
            setFullStrip(RGB.WHITE, false);
            break;
        case WAYPOINT:
            setFullStrip(RGB.YELLOW, false);
            break;
          case CORAL_MODE:
            setFullStrip(RGB.BLUE, false);
            break;
          case CORAL_INTAKE:
            setFullStrip(RGB.BLUE, true);
            break;
          case ALGAE_MODE:
            setFullStrip(RGB.GREEN, false);
            break;
          case ALGAE_INTAKE:
            setFullStrip(RGB.GREEN, true);
            break;
          case REEF:
            setFullStrip(RGB.PURPLE, false);
            break;
          default:
            break;
        }
        return;
    }

    public Command setLights(int id) {
        switch (id) {
            case 0:
                return runOnce(() -> setState(LightState.IDLE));
            case 1:
                return runOnce(() -> setState(LightState.WAYPOINT));
            case 2:
                return runOnce(() -> setState(LightState.CORAL_MODE));
            case 3:
                return runOnce(() -> setState(LightState.CORAL_INTAKE));
            case 4:
                return runOnce(() -> setState(LightState.ALGAE_MODE));
            case 5:
                return runOnce(() -> setState(LightState.ALGAE_INTAKE));
            case 6:
                return runOnce(() -> setState(LightState.REEF));
            default:
                return runOnce(() -> setState(LightState.IDLE));
        }
    }

    private void setFullStrip(final RGB rgb, boolean flash) {
        Logger.recordOutput("Signaling/LED Color", rgb.toHex());
        clearAnimation();
        if (flash) {
            setAnimation(new StrobeAnimation(rgb.red, rgb.green, rgb.blue, 0, 0.5, SignalingConstants.NUMBER_OF_LEDS));
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
}
