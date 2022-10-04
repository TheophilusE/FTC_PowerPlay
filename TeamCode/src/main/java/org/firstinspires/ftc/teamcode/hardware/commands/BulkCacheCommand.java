package org.firstinspires.ftc.teamcode.hardware.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/*
 * Command that runs infinitely and will constantly clear the bulk cache each loop.
 */
public class BulkCacheCommand extends CommandBase
{
  private final List<LynxModule> detectedHubs;

  // Construct
  public BulkCacheCommand(HardwareMap hardwareMap)
  {
    detectedHubs = hardwareMap.getAll(LynxModule.class);
  }

  @Override
  public void initialize()
  {
    detectedHubs.forEach((hub) ->
                         {
                           hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
                         });
  }

  @Override
  public void execute()
  {
    detectedHubs.forEach(LynxModule::clearBulkCache);
  }
}
