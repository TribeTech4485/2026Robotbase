package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.SyncedLibraries.SystemBases.LedBase;

public class LEDS extends LedBase {
  LEDPattern elevatorPatt;
  LEDPattern robotBasePattern;
  LEDPattern robotBaseDisabledPattern;
  LEDPattern whitePattern;

  Elevator ele;

  AddressableLEDBufferView robotBaseView;
  AddressableLEDBufferView blank1View;
  AddressableLEDBufferView topUpView;
  AddressableLEDBufferView topDownView;
  AddressableLEDBufferView blank2View;

  static final int length = 295; // 297
  static final int baseLength = 145; // 147
  static final int blank1Length = 2; // 13
  static final int blank2Length = 25; // 25
  static final int topUpLength = (length - baseLength - blank1Length - blank2Length) / 2;
  static final int topDownLength = topUpLength;

  static final int blank1Start = baseLength;
  static final int topUpStart = blank1Start + blank1Length;
  static final int topDownStart = topUpStart + topUpLength;
  static final int blank2Start = topDownStart + topDownLength;

  public LEDS(Elevator ele) {
    super(1, length);

    this.ele = ele;

    robotBaseView = buffer.createView(0, baseLength);
    blank1View = buffer.createView(blank1Start, blank1Start + blank1Length);
    topUpView = buffer.createView(topUpStart, topUpStart + topUpLength).reversed();
    topDownView = buffer.createView(topDownStart, topDownStart + topDownLength);
    blank2View = buffer.createView(blank2Start, blank2Start + blank2Length);

    // System.out.println("RobotBAse start " + 0 + " long " + baseLength);
    // System.out.println("Blank1 start " + blank1Start + " long " + blank1Length);
    // System.out.println("TopUp start " + topUpStart + " long " + topUpLength);
    // System.out.println("TopDown start " + topDownStart + " long " +
    // topDownLength);
    // System.out.println("Blank2 start " + blank2Start + " long " + blank2Length);

    // double baseBrightness = 25;
    double heightBrightness = 100 / 3;
    LEDPattern elevatorBasePattern = LEDPattern.rainbow(255, 255)
        .scrollAtAbsoluteSpeed(FeetPerSecond.of(-4), spacing);

    // LEDPattern softAllianceTone = LEDPattern.solid(getAllianceColor())
    // .blend(LEDPattern.solid(Color.kWhite));

    // bottom to to elevator height
    LEDPattern elevatorHightMasked = elevatorBasePattern
        .mask(LEDPattern.progressMaskLayer(() -> 1 - ele.getPosPercent()))
        .atBrightness(Percent.of(heightBrightness));
    // top to elevator height
    // LEDPattern elevatorBaseReverseMasked = LEDPattern.solid(getAllianceColor())
    // .mask(LEDPattern.progressMaskLayer(() -> ele.getPosPercent()))
    // .breathe(Seconds.of(3))
    // .atBrightness(Percent.of(baseBrightness));

    // elevatorPatt = elevatorHightMasked.overlayOn(elevatorBaseReverseMasked);
    elevatorPatt = elevatorHightMasked;
    // LEDPattern robotEnabledOnFieldMask = LEDPattern.progressMaskLayer(
    // () -> (DriverStation.isEnabled()) ? 0 : 1);
    // robotBasePattern = robotBasePattern.mask(robotEnabledOnFieldMask);

    robotBaseDisabledPattern = LEDPattern.gradient(GradientType.kContinuous, Color.kGreen, new Color(0, 20, 0))
        .scrollAtAbsoluteSpeed(FeetPerSecond.of(10), spacing)
        .atBrightness(Percent.of(75));
    // .breathe(Seconds.of(2.5));
    // .blend(LEDPattern.solid(Color.kGreen).atBrightness(Percent.of(25)));

    whitePattern = LEDPattern.gradient(GradientType.kContinuous, Color.kWhite, Color.kGray)
        .atBrightness(Percent.of(75)).scrollAtAbsoluteSpeed(FeetPerSecond.of(-2), spacing);

    redoBasePattern();
    // reload the colors on init
    Robot.onInits.add(new InstantCommand(this::redoBasePattern));
  }

  private void redoBasePattern() {
    robotBasePattern = LEDPattern.solid(getAllianceColor())
        .atBrightness(Percent.of(75))
        .breathe(Seconds.of(1.5));
  }

  @Override
  protected void applyPatterns() {
    if (ele.getPosPercent() > 0.75 && ele.getVelocity().abs(MetersPerSecond) < 0.25) {
      // If the elevator is at the top, set all LEDs to white
      whitePattern.applyTo(topUpView);
      whitePattern.applyTo(topDownView);
      LEDPattern.kOff.applyTo(robotBaseView);
      LEDPattern.kOff.applyTo(blank1View);
      LEDPattern.kOff.applyTo(blank2View);
    } else {
      elevatorPatt.applyTo(topUpView);
      elevatorPatt.applyTo(topDownView);
      if (!DriverStation.isDisabled()) {
        robotBasePattern.applyTo(robotBaseView);
        robotBasePattern.applyTo(blank1View);
        robotBasePattern.applyTo(blank2View);
      } else {
        robotBaseDisabledPattern.applyTo(robotBaseView);
        if (DriverStation.isDSAttached()) {
          LEDPattern.kOff.applyTo(blank1View);
          LEDPattern.kOff.applyTo(blank2View);
        } else {
          // no ds add white to the blank spots
          whitePattern.applyTo(blank1View);
          whitePattern.applyTo(blank2View);
        }
      }
    }
  }
}