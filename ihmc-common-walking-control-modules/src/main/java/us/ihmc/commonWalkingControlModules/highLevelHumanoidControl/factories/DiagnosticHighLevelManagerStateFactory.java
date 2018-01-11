package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.DiagnosticHighLevelManagerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class DiagnosticHighLevelManagerStateFactory implements HighLevelControllerStateFactory
{
   private DiagnosticHighLevelManagerState diagnosticHighLevelManagerState;
   private boolean isTransitionRequested = false;

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (diagnosticHighLevelManagerState == null)
         diagnosticHighLevelManagerState = new DiagnosticHighLevelManagerState(controllerFactoryHelper.getHighLevelControllerParameters(),
                                                                               controllerFactoryHelper.getHighLevelHumanoidControllerToolbox());

      return diagnosticHighLevelManagerState;
   }

   public void setTransitionRequested(boolean isTransitionRequested)
   {
      this.isTransitionRequested = isTransitionRequested;
   }

   @Override
   public boolean isTransitionToControllerRequested()
   {
      return isTransitionRequested;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.DIAGNOSTICS;
   }
}
