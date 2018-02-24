package us.ihmc.humanoidBehaviors.dispatcher;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.BehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.BehaviorStateMachine;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionAction;
import us.ihmc.sensorProcessing.communication.subscribers.RobotDataReceiver;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * The BehaviorDispatcher is used to select the behavior to run and to execute operator's commands as pause, resume, stop, etc.
 * DO NOT add smart AI stuff in there, create and register a new behavior in {@link IHMCHumanoidBehaviorManager} instead.
 *
 */
public class BehaviorDispatcher<E extends Enum<E>> implements Runnable
{
   private static final boolean DEBUG = true;
   private final Class<E> behaviorEnum;
   private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("BehaviorDispatcher"));

   private final String name = getClass().getSimpleName();

   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final YoDouble yoTime;
   private final YoVariableServer yoVaribleServer;
   private final BehaviorStateMachine<E> stateMachine;

   private final YoEnum<E> requestedBehavior;

   private final BehaviorTypeSubscriber<E> desiredBehaviorSubscriber;
   private final BehaviorControlModeSubscriber desiredBehaviorControlSubscriber;
   private final CommunicationBridge communicationBridge;

   private final RobotDataReceiver robotDataReceiver;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final ArrayList<Updatable> updatables = new ArrayList<>();

   private final YoBoolean hasBeenInitialized = new YoBoolean("hasBeenInitialized", registry);

   private E stopBehavior;
   private E currentBehavior;

   public BehaviorDispatcher(YoDouble yoTime, RobotDataReceiver robotDataReceiver, BehaviorControlModeSubscriber desiredBehaviorControlSubscriber,
         BehaviorTypeSubscriber<E> desiredBehaviorSubscriber, CommunicationBridge communicationBridge, YoVariableServer yoVaribleServer, Class<E> behaviourEnum,
         E stopBehavior, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.behaviorEnum = behaviourEnum;
      this.stopBehavior = stopBehavior;
      this.yoTime = yoTime;
      this.yoVaribleServer = yoVaribleServer;
      this.communicationBridge = communicationBridge;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.requestedBehavior = new YoEnum<E>("requestedBehavior", registry, behaviourEnum, true);

      this.robotDataReceiver = robotDataReceiver;
      this.desiredBehaviorSubscriber = desiredBehaviorSubscriber;
      this.desiredBehaviorControlSubscriber = desiredBehaviorControlSubscriber;

      stateMachine = new BehaviorStateMachine<E>("behaviorState", "behaviorSwitchTime", behaviourEnum, yoTime, registry);

      SimpleDoNothingBehavior simpleForwardingBehavior = new SimpleDoNothingBehavior(communicationBridge);
      addBehavior(stopBehavior, simpleForwardingBehavior);
      stateMachine.setCurrentState(stopBehavior);

      requestedBehavior.set(null);

      parentRegistry.addChild(registry);
   }

   public void requestBehavior(E behaviorEnum)
   {
      requestedBehavior.set(behaviorEnum);
   }

   public void addBehaviors(List<E> Es, List<AbstractBehavior> newBehaviors)
   {
      if (Es.size() != newBehaviors.size())
         throw new RuntimeException("Arguments don't have the same size.");

      for (int i = 0; i < Es.size(); i++)
      {
         addBehavior(Es.get(i), newBehaviors.get(i));
      }
   }

   public void addBehavior(E E, AbstractBehavior behaviorToAdd)
   {
      BehaviorAction<E> behaviorStateToAdd = new BehaviorAction<E>(E, behaviorToAdd);

      this.stateMachine.addState(behaviorStateToAdd);
      try
      {
         this.registry.addChild(behaviorToAdd.getYoVariableRegistry());
      }
      catch (Exception e)
      {
         PrintTools.info(e.getMessage());
      }

      ArrayList<BehaviorAction<E>> allOtherBehaviorStates = new ArrayList<BehaviorAction<E>>();

      for (E otherBehaviorType : behaviorEnum.getEnumConstants())
      {
         BehaviorAction<E> otherBehaviorState = stateMachine.getState(otherBehaviorType);

         if (otherBehaviorState == null)
         {
            continue;
         }
         else
         {
            allOtherBehaviorStates.add(otherBehaviorState);

            boolean waitUntilDone = false;

            SwitchGlobalListenersAction switchToOtherBehaviorState = new SwitchGlobalListenersAction(behaviorStateToAdd, otherBehaviorState);
            StateMachineTools.addRequestedStateTransition(requestedBehavior, waitUntilDone, switchToOtherBehaviorState, behaviorStateToAdd, otherBehaviorState);

            SwitchGlobalListenersAction switchFromOtherBehaviorState = new SwitchGlobalListenersAction(otherBehaviorState, behaviorStateToAdd);
            StateMachineTools.addRequestedStateTransition(requestedBehavior, waitUntilDone, switchFromOtherBehaviorState, otherBehaviorState,
                  behaviorStateToAdd);
         }
      }
   }

   public void addBehaviorService(BehaviorService behaviorService)
   {
      registry.addChild(behaviorService.getYoVariableRegistry());
   }

   private void initialize()
   {
      BehaviorAction<E> currentState = stateMachine.getCurrentState();
      currentState.doTransitionIntoAction();
   }

   private void doControl()
   {
      updateRobotState();
      updateControlStatus();
      updateRequestedBehavior();
      callUpdatables();


      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      //a behavior has finished or has aborted and has transitioned to STOP


      if (stateMachine.getCurrentStateEnum().equals(stopBehavior) && currentBehavior!=null && !currentBehavior.equals(stopBehavior))
      {
         communicationBridge.sendPacketToUI(HumanoidMessageTools.createBehaviorStatusPacket(CurrentBehaviorStatus.NO_BEHAVIOR_RUNNING));
      }
      currentBehavior = stateMachine.getCurrentStateEnum();

      yoGraphicsListRegistry.update();
   }

   private void callUpdatables()
   {
      for (int i = 0; i < updatables.size(); i++)
      {
         updatables.get(i).update(yoTime.getDoubleValue());
      }
   }

   public void addUpdatable(Updatable updatable)
   {
      updatables.add(updatable);
   }

   private void updateRobotState()
   {
      robotDataReceiver.updateRobotModel();
      long simTimestamp = robotDataReceiver.getSimTimestamp();

      if (simTimestamp < 0)
         return;

      double currentTimeInSeconds = Conversions.nanosecondsToSeconds(simTimestamp);
      yoTime.set(currentTimeInSeconds);
   }

   private void updateRequestedBehavior()
   {
      if (desiredBehaviorSubscriber.checkForNewBehaviorRequested())
      {
         requestedBehavior.set(desiredBehaviorSubscriber.getRequestedBehavior());
      }
   }

   private void updateControlStatus()
   {
      if (desiredBehaviorControlSubscriber.checkForNewControlRequested())
      {
         switch (desiredBehaviorControlSubscriber.getRequestedBehaviorControl())
         {
         case STOP:
            stateMachine.stop();
            communicationBridge.sendPacketToUI(HumanoidMessageTools.createBehaviorStatusPacket(CurrentBehaviorStatus.NO_BEHAVIOR_RUNNING));
            communicationBridge.sendPacket(HumanoidMessageTools.createBehaviorControlModeResponsePacket(BehaviorControlModeEnum.STOP));
            break;
         case PAUSE:
            stateMachine.pause();
            communicationBridge.sendPacketToUI(HumanoidMessageTools.createBehaviorStatusPacket(CurrentBehaviorStatus.BEHAVIOR_PAUSED));

            communicationBridge.sendPacket(HumanoidMessageTools.createBehaviorControlModeResponsePacket(BehaviorControlModeEnum.PAUSE));
            break;
         case RESUME:
            stateMachine.resume();
            communicationBridge.sendPacketToUI(HumanoidMessageTools.createBehaviorStatusPacket(CurrentBehaviorStatus.BEHAVIOS_RUNNING));

            communicationBridge.sendPacket(HumanoidMessageTools.createBehaviorControlModeResponsePacket(BehaviorControlModeEnum.RESUME));
            break;
         default:
            throw new IllegalArgumentException("BehaviorCommunicationBridge, unhandled control!");
         }

      }
   }

   @Override
   public void run()
   {
      try
      {
         if (!hasBeenInitialized.getBooleanValue())
         {
            initialize();
            hasBeenInitialized.set(true);
         }

         doControl();

         if (yoVaribleServer != null)
         {
            yoVaribleServer.update(Conversions.secondsToNanoseconds(yoTime.getDoubleValue()));
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private class SwitchGlobalListenersAction implements StateTransitionAction
   {
      private final BehaviorAction<E> fromBehaviorState;
      private final BehaviorAction<E> toBehaviorState;

      public SwitchGlobalListenersAction(BehaviorAction<E> fromBehaviorState, BehaviorAction<E> toBehaviorState)
      {
         this.fromBehaviorState = fromBehaviorState;
         this.toBehaviorState = toBehaviorState;
      }

      @Override
      public void doTransitionAction()
      {

      }
   }

   public void start()
   {
      // do start
      scheduler.scheduleAtFixedRate(this, 0, 10, TimeUnit.MILLISECONDS);
   }

   public void closeAndDispose()
   {

      // do stop
      scheduler.shutdown();
      try
      {
         scheduler.awaitTermination(10, TimeUnit.SECONDS);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException("Cannot shutdown BehaviorDispatcher", e);
      }
   }
}
