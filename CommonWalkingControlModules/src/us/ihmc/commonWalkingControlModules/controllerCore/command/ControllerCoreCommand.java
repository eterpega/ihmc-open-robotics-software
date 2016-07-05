package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;

public class ControllerCoreCommand implements ControllerCoreCommandInterface
{
   private final InverseDynamicsCommandList inverseDynamicsCommandList;
   private final InverseDynamicsCommandList virtualModelControlCommandList;
   private final FeedbackControlCommandList feedbackControlCommandList;
   private final InverseKinematicsCommandList inverseKinematicsCommandList;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataToAdd;
   private WholeBodyControllerCoreMode controllerCoreMode;

   public ControllerCoreCommand(WholeBodyControllerCoreMode controllerCoreMode)
   {
      this.controllerCoreMode = controllerCoreMode;

      inverseDynamicsCommandList = new InverseDynamicsCommandList();
      virtualModelControlCommandList = new InverseDynamicsCommandList();
      feedbackControlCommandList = new FeedbackControlCommandList();
      inverseKinematicsCommandList = new InverseKinematicsCommandList();
      lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
      lowLevelOneDoFJointDesiredDataToAdd = new LowLevelOneDoFJointDesiredDataHolder();
   }

   public void clear()
   {
      inverseDynamicsCommandList.clear();
      feedbackControlCommandList.clear();
      inverseKinematicsCommandList.clear();
      lowLevelOneDoFJointDesiredDataHolder.clear();
      lowLevelOneDoFJointDesiredDataToAdd.clear();
   }

   public void addInverseDynamicsCommand(InverseDynamicsCommand<?> inverseDynamicsCommand)
   {
      if (inverseDynamicsCommand != null)
         inverseDynamicsCommandList.addCommand(inverseDynamicsCommand);
   }

   public void addVirtualModelControlCommand(InverseDynamicsCommand<?> inverseDynamicsCommand)
   {
      if (inverseDynamicsCommand != null)
         virtualModelControlCommandList.addCommand(inverseDynamicsCommand);
   }

   public void addFeedbackControlCommand(FeedbackControlCommand<?> feedbackControlCommand)
   {
      if (feedbackControlCommand != null)
         feedbackControlCommandList.addCommand(feedbackControlCommand);
   }

   public void addInverseKinematicsCommand(InverseKinematicsCommand<?> inverseKinematicsCommand)
   {
      if (inverseKinematicsCommand != null)
         inverseKinematicsCommandList.addCommand(inverseKinematicsCommand);
   }

   public void completeLowLevelJointData(LowLevelOneDoFJointDesiredDataHolderReadOnly lowLevelJointData)
   {
      if (lowLevelJointData != null)
         lowLevelOneDoFJointDesiredDataHolder.completeWith(lowLevelJointData);
   }

   public void addLowLevelJointData(LowLevelOneDoFJointDesiredDataHolderReadOnly lowLevelJointDataToAdd)
   {
      if (lowLevelJointDataToAdd != null)
         lowLevelOneDoFJointDesiredDataToAdd.add(lowLevelJointDataToAdd);
   }

   public void setControllerCoreMode(WholeBodyControllerCoreMode controllerCoreMode)
   {
      if (this.controllerCoreMode != controllerCoreMode)
      {
         clear();
         this.controllerCoreMode = controllerCoreMode;
      }
   }

   @Override
   public InverseDynamicsCommandList getInverseDynamicsCommandList()
   {
      return inverseDynamicsCommandList;
   }

   @Override
   public InverseDynamicsCommandList getVirtualModelControlCommandList()
   {
      return virtualModelControlCommandList;
   }

   @Override
   public FeedbackControlCommandList getFeedbackControlCommandList()
   {
      return feedbackControlCommandList;
   }

   @Override
   public InverseKinematicsCommandList getInverseKinematicsCommandList()
   {
      return inverseKinematicsCommandList;
   }

   @Override
   public LowLevelOneDoFJointDesiredDataHolder getLowLevelOneDoFJointDesiredDataHolder()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   @Override
   public LowLevelOneDoFJointDesiredDataHolder getLowLevelOneDoFJointDesiredDataToAdd()
   {
      return lowLevelOneDoFJointDesiredDataToAdd;
   }

   public void set(ControllerCoreCommand other)
   {
      controllerCoreMode = other.controllerCoreMode;
      inverseDynamicsCommandList.set(other.inverseDynamicsCommandList);
      feedbackControlCommandList.set(other.feedbackControlCommandList);
      inverseKinematicsCommandList.set(other.inverseKinematicsCommandList);
      lowLevelOneDoFJointDesiredDataHolder.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
      lowLevelOneDoFJointDesiredDataToAdd.overwriteWith(other.lowLevelOneDoFJointDesiredDataToAdd);
   }

   @Override
   public WholeBodyControllerCoreMode getControllerCoreMode()
   {
      return controllerCoreMode;
   }
}
