package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerCommandInputManager;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHighLevelStateMessage;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.ControllerCoreOuput;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangeMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.GenericStateMachine;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateChangedListener;
import us.ihmc.robotics.stateMachines.StateMachineTools;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureListener;

public class HighLevelHumanoidControllerManager implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final GenericStateMachine<HighLevelState, HighLevelBehavior> stateMachine;
   private final HighLevelState initialBehavior;
   private final MomentumBasedController momentumBasedController;

   private final EnumYoVariable<HighLevelState> requestedHighLevelState = new EnumYoVariable<HighLevelState>("requestedHighLevelState", registry,
         HighLevelState.class, true);

   private final BooleanYoVariable isListeningToHighLevelStateMessage = new BooleanYoVariable("isListeningToHighLevelStateMessage", registry);

   private final CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator;
   private final ControllerCommandInputManager commandInputManager;
   private final ControllerCoreOuput controllerCoreOuput;
   private final WholeBodyControllerCore controllerCore;

   private final AtomicReference<HighLevelState> fallbackControllerForFailureReference = new AtomicReference<>();

   public static final AtomicReference<HighLevelState> currentStateForOpenSource = new AtomicReference<HighLevelState>(HighLevelState.DO_NOTHING_BEHAVIOR);

   public HighLevelHumanoidControllerManager(ControllerCommandInputManager commandInputManager, WholeBodyControllerCore controllerCore,
         HighLevelState initialBehavior, ArrayList<HighLevelBehavior> highLevelBehaviors, MomentumBasedController momentumBasedController,
         CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator, ControllerCoreOuput controllerCoreOuput, HumanoidGlobalDataProducer dataProducer)
   {
      this.commandInputManager = commandInputManager;
      DoubleYoVariable yoTime = momentumBasedController.getYoTime();
      this.controllerCoreOuput = controllerCoreOuput;
      this.controllerCore = controllerCore;

      this.stateMachine = setUpStateMachine(highLevelBehaviors, yoTime, registry, dataProducer);
      requestedHighLevelState.set(initialBehavior);

      isListeningToHighLevelStateMessage.set(true);

      for (int i = 0; i < highLevelBehaviors.size(); i++)
      {
         this.registry.addChild(highLevelBehaviors.get(i).getYoVariableRegistry());
      }
      this.initialBehavior = initialBehavior;
      this.momentumBasedController = momentumBasedController;
      this.centerOfPressureDataHolderForEstimator = centerOfPressureDataHolderForEstimator;
      this.registry.addChild(momentumBasedController.getYoVariableRegistry());

      momentumBasedController.attachControllerFailureListener(new ControllerFailureListener()
      {
         @Override
         public void controllerFailed(FrameVector2d fallingDirection)
         {
            HighLevelState fallbackController = fallbackControllerForFailureReference.get();
            if (fallbackController != null)
               requestedHighLevelState.set(fallbackController);
         }
      });
   }

   public void setFallbackControllerForFailure(HighLevelState fallbackController)
   {
      fallbackControllerForFailureReference.set(fallbackController);
   }

   private GenericStateMachine<HighLevelState, HighLevelBehavior> setUpStateMachine(ArrayList<HighLevelBehavior> highLevelBehaviors, DoubleYoVariable yoTime,
         YoVariableRegistry registry, final HumanoidGlobalDataProducer dataProducer)
   {
      GenericStateMachine<HighLevelState, HighLevelBehavior> highLevelStateMachine = new GenericStateMachine<>("highLevelState", "switchTimeName",
            HighLevelState.class, yoTime, registry);

      // Enable transition between every existing state of the state machine
      for (int i = 0; i < highLevelBehaviors.size(); i++)
      {
         HighLevelBehavior highLevelStateA = highLevelBehaviors.get(i);
         highLevelStateA.setControllerCoreOuput(controllerCoreOuput);

         for (int j = 0; j < highLevelBehaviors.size(); j++)
         {
            HighLevelBehavior highLevelStateB = highLevelBehaviors.get(j);

            StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, highLevelStateA, highLevelStateB);
            StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, highLevelStateB, highLevelStateA);
         }
      }

      for (int i = 0; i < highLevelBehaviors.size(); i++)
      {
         highLevelStateMachine.addState(highLevelBehaviors.get(i));
      }

      highLevelStateMachine.attachStateChangedListener(new StateChangedListener<HighLevelState>()
      {
         @Override
         public void stateChanged(State<HighLevelState> oldState, State<HighLevelState> newState, double time)
         {
            currentStateForOpenSource.set(newState.getStateEnum());
            if (dataProducer != null)
               dataProducer.queueDataToSend(new HighLevelStateChangeMessage(oldState.getStateEnum(), newState.getStateEnum()));
         }
      });

      return highLevelStateMachine;
   }

   public void addYoVariableRegistry(YoVariableRegistry registryToAdd)
   {
      this.registry.addChild(registryToAdd);
   }

   public void requestHighLevelState(HighLevelState requestedHighLevelState)
   {
      this.requestedHighLevelState.set(requestedHighLevelState);
   }

   public void setListenToHighLevelStatePackets(boolean isListening)
   {
      isListeningToHighLevelStateMessage.set(isListening);
   }

   public void initialize()
   {
      controllerCore.initialize();
      momentumBasedController.initialize();
      stateMachine.setCurrentState(initialBehavior);
   }

   public void doControl()
   {
      if (isListeningToHighLevelStateMessage.getBooleanValue())
      {
         if (commandInputManager.isNewMessageAvailable(ModifiableHighLevelStateMessage.class))
         {
            requestedHighLevelState.set(commandInputManager.pollNewestMessage(ModifiableHighLevelStateMessage.class).getHighLevelState());
         }
      }

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      ControllerCoreCommand controllerCoreCommandList = stateMachine.getCurrentState().getControllerCoreCommand();
      controllerCore.submitControllerCoreCommand(controllerCoreCommandList);
      controllerCore.compute();
      reportDesiredCenterOfPressureForEstimator();
   }

   private final SideDependentList<FramePoint2d> desiredFootCoPs = new SideDependentList<FramePoint2d>(new FramePoint2d(), new FramePoint2d());

   private void reportDesiredCenterOfPressureForEstimator()
   {
      SideDependentList<? extends ContactablePlaneBody> contactableFeet = momentumBasedController.getContactableFeet();
      FullHumanoidRobotModel fullHumanoidRobotModel = momentumBasedController.getFullRobotModel();
      for (RobotSide robotSide : RobotSide.values)
      {
         momentumBasedController.getDesiredCenterOfPressure(contactableFeet.get(robotSide), desiredFootCoPs.get(robotSide));
         centerOfPressureDataHolderForEstimator.setCenterOfPressure(desiredFootCoPs.get(robotSide), fullHumanoidRobotModel.getFoot(robotSide));
      }
   }

   public void addHighLevelBehavior(HighLevelBehavior highLevelBehavior, boolean transitionRequested)
   {
      highLevelBehavior.setControllerCoreOuput(controllerCoreOuput);

      // Enable transition between every existing state of the state machine
      for (HighLevelState stateEnum : HighLevelState.values)
      {
         State<HighLevelState> otherHighLevelState = stateMachine.getState(stateEnum);
         if (otherHighLevelState == null)
            continue;

         StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, otherHighLevelState, highLevelBehavior);
         StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, highLevelBehavior, otherHighLevelState);
      }

      this.stateMachine.addState(highLevelBehavior);
      this.registry.addChild(highLevelBehavior.getYoVariableRegistry());

      if (transitionRequested)
         requestedHighLevelState.set(highLevelBehavior.getStateEnum());
   }

   public HighLevelState getCurrentHighLevelState()
   {
      return stateMachine.getCurrentStateEnum();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return this.getClass().getSimpleName();
   }

   public String getDescription()
   {
      return getName();
   }

}
