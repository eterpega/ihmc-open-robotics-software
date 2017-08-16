package us.ihmc.atlas.atlasRosControl;

import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.wholeBodyController.DRCOutputWriter;

public class AtlasRosControlOutputWriter implements DRCOutputWriter, ControllerStateChangedListener, ControllerFailureListener
{

   public AtlasRosControlOutputWriter(AtlasRobotModel robotModel)
   {
      // TODO Auto-generated constructor stub
   }

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void writeAfterController(long timestamp)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setFullRobotModel(FullHumanoidRobotModel controllerModel, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForController)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void controllerStateHasChanged(Enum<?> oldState, Enum<?> newState)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void controllerFailed(FrameVector2D fallingDirection)
   {
      // TODO Auto-generated method stub
      
   }

}
