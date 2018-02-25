package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.mechanics.virtualModelController.QuadrupedVirtualModelController;
import us.ihmc.quadrupedRobotics.mechanics.virtualModelController.QuadrupedVirtualModelControllerSettings;
import us.ihmc.quadrupedRobotics.optimization.contactForceOptimization.QuadrupedContactForceLimits;
import us.ihmc.quadrupedRobotics.optimization.contactForceOptimization.QuadrupedContactForceOptimization;
import us.ihmc.quadrupedRobotics.optimization.contactForceOptimization.QuadrupedContactForceOptimizationSettings;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTaskSpaceController
{
   public static class Settings
   {
      private final QuadrantDependentList<ContactState> contactState;
      private final QuadrupedContactForceLimits contactForceLimits;
      private final QuadrupedContactForceOptimizationSettings contactForceOptimizationSettings;
      private final QuadrupedVirtualModelControllerSettings virtualModelControllerSettings;

      public Settings()
      {
         contactState = new QuadrantDependentList<>();
         contactForceLimits = new QuadrupedContactForceLimits();
         contactForceOptimizationSettings = new QuadrupedContactForceOptimizationSettings();
         virtualModelControllerSettings = new QuadrupedVirtualModelControllerSettings();
         initialize();
      }

      public void initialize()
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            contactState.set(robotQuadrant, ContactState.NO_CONTACT);
         }
         contactForceLimits.setDefaults();
         contactForceOptimizationSettings.setDefaults();
         virtualModelControllerSettings.setDefaults();
      }

      public void setContactState(RobotQuadrant robotQuadrant, ContactState contactState)
      {
         this.contactState.set(robotQuadrant, contactState);
      }

      public void setContactState(QuadrantDependentList<ContactState> contactState)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            this.contactState.set(robotQuadrant, contactState.get(robotQuadrant));
         }
      }

      public ContactState getContactState(RobotQuadrant robotQuadrant)
      {
         return contactState.get(robotQuadrant);
      }

      public QuadrantDependentList<ContactState> getContactState()
      {
         return contactState;
      }

      public QuadrupedContactForceLimits getContactForceLimits()
      {
         return contactForceLimits;
      }

      public QuadrupedContactForceOptimizationSettings getContactForceOptimizationSettings()
      {
         return contactForceOptimizationSettings;
      }

      public QuadrupedVirtualModelControllerSettings getVirtualModelControllerSettings()
      {
         return virtualModelControllerSettings;
      }
   }

   public static class Commands
   {
      private final FrameVector3D comForce = new FrameVector3D();
      private final FrameVector3D comTorque = new FrameVector3D();
      private final QuadrantDependentList<FrameVector3D> soleForce = new QuadrantDependentList<>();

      public Commands()
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            soleForce.set(robotQuadrant, new FrameVector3D());
         }
         initialize();
      }

      public void initialize()
      {
         comForce.setToZero();
         comTorque.setToZero();
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            soleForce.get(robotQuadrant).setToZero();
         }
      }

      public FrameVector3D getComForce()
      {
         return comForce;
      }

      public FrameVector3D getComTorque()
      {
         return comTorque;
      }

      public FrameVector3D getSoleForce(RobotQuadrant robotQuadrant)
      {
         return soleForce.get(robotQuadrant);
      }

      public QuadrantDependentList<FrameVector3D> getSoleForce()
      {
         return soleForce;
      }
   }

   private final QuadrupedVirtualModelController virtualModelController;
   private final QuadrupedContactForceOptimization contactForceOptimization;
   private final FrameVector3D contactForceStorage;
   private final YoVariableRegistry registry = new YoVariableRegistry("taskSpaceController");
   private final YoLong contactForceOptimizationSolveTime = new YoLong("contactForceOptimizationSolveTime", registry);

   public QuadrupedTaskSpaceController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedReferenceFrames referenceFrames,
                                       YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      // virtual model controller
      virtualModelController = new QuadrupedVirtualModelController(runtimeEnvironment, referenceFrames, registry, graphicsListRegistry);
      contactForceOptimization = new QuadrupedContactForceOptimization(referenceFrames, registry);
      contactForceStorage = new FrameVector3D();

      parentRegistry.addChild(registry);
      reset();
   }

   public void reset()
   {
      virtualModelController.reset();
      contactForceOptimization.reset();
   }

   public void compute(Settings settings, Commands commands)
   {
      // compute optimal contact force distribution for quadrants that are in contact
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         // note: sole forces are inverted to obtain commanded reaction forces
         commands.getSoleForce().get(robotQuadrant).scale(-1.0);
         contactForceOptimization.setContactForceCommand(robotQuadrant, commands.getSoleForce().get(robotQuadrant));
         commands.getSoleForce().get(robotQuadrant).scale(-1.0);
         contactForceOptimization.setContactState(robotQuadrant, settings.getContactState(robotQuadrant));
      }
      contactForceOptimization.setComForceCommand(commands.getComForce());
      contactForceOptimization.setComTorqueCommand(commands.getComTorque());
      long timeContactForceOptimizationSolverStart = System.nanoTime();
      contactForceOptimization.solve(settings.getContactForceLimits(), settings.getContactForceOptimizationSettings());
      long timeContactForceOptimizationSolverEnds = System.nanoTime();
      contactForceOptimizationSolveTime.set(timeContactForceOptimizationSolverEnds - timeContactForceOptimizationSolverStart);
      
      // compute leg joint torques using jacobian transpose
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (settings.getContactState(robotQuadrant) == ContactState.IN_CONTACT)
         {
            contactForceOptimization.getContactForceSolution(robotQuadrant, contactForceStorage);
            virtualModelController.setSoleContactForce(robotQuadrant, contactForceStorage);
            virtualModelController.setSoleContactForceVisible(robotQuadrant, true);
            virtualModelController.setSoleVirtualForceVisible(robotQuadrant, false);
         }
         else
         {
            virtualModelController.setSoleVirtualForce(robotQuadrant, commands.getSoleForce(robotQuadrant));
            virtualModelController.setSoleContactForceVisible(robotQuadrant, false);
            virtualModelController.setSoleVirtualForceVisible(robotQuadrant, true);
         }
         virtualModelController.setJointTorquesVisible(robotQuadrant, false);
      }
      virtualModelController.compute(settings.getVirtualModelControllerSettings());
   }

   public ContactState getContactState(RobotQuadrant robotQuadrant)
   {
      return contactForceOptimization.getContactState(robotQuadrant);
   }
}
