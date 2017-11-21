package us.ihmc.exampleSimulations.simple3dofBiped;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.geometry.interfaces.PointInterface;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.SimpleState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleLeapOfFaithHeuristicController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getName());
   private final Simple3dofBipedRobot robot;

   private final SideDependentList<PDController> hipPDControllers = new SideDependentList<>();
   private final SideDependentList<PDController> kneePDControllers = new SideDependentList<>();

   private final SideDependentList<YoDouble> q_d_hips = new SideDependentList<>();
   private final SideDependentList<YoDouble> q_d_knees = new SideDependentList<>();

   private final YoDouble previousTickTime = new YoDouble("previousTickTime", registry);
   private final YoDouble deltaTime = new YoDouble("deltaTime", registry);

   private final YoDouble totalMass = new YoDouble("totalMass", registry);
   private final YoDouble minimumKneeForce = new YoDouble("minimumKneeForce", registry);
   private final YoDouble maximumKneeForce = new YoDouble("maximumKneeForce", registry);

   private final YoDouble toeOffKneeVelocity = new YoDouble("toeOffKneeVelocity", registry);
   private final YoDouble swingTime = new YoDouble("swingTime", registry);
   private final YoDouble retractKneeForSwingVelocity = new YoDouble("retractKneeForSwingVelocity", registry);
      
   private final StateMachine<LeapOfFaithState> stateMachine;

   public SimpleLeapOfFaithHeuristicController(Simple3dofBipedRobot simple3dofBiped)
   {
      this.robot = simple3dofBiped;      
      
      for (RobotSide robotSide : RobotSide.values)
      {
         PDController hipPDController = new PDController(robotSide.getCamelCaseNameForStartOfExpression() + "HipPDController", registry);
         hipPDControllers.set(robotSide, hipPDController);

         PDController kneePDController = new PDController(robotSide.getCamelCaseNameForStartOfExpression() + "KneePDController", registry);
         kneePDControllers.set(robotSide, kneePDController);

         YoDouble q_d_hip = new YoDouble("q_d_" + robotSide.getCamelCaseNameForMiddleOfExpression() + "Hip", registry);
         YoDouble q_d_knee = new YoDouble("q_d_" + robotSide.getCamelCaseNameForMiddleOfExpression() + "Knee", registry);

         q_d_hips.set(robotSide, q_d_hip);
         q_d_knees.set(robotSide, q_d_knee);
      }

      Point3D comPoint = new Point3D();
      totalMass.set(simple3dofBiped.computeCenterOfMass(comPoint));

      
      retractKneeForSwingVelocity.set(0.5);

      minimumKneeForce.set(-totalMass.getDoubleValue() * 9.81 * 0.2);
      maximumKneeForce.set(totalMass.getDoubleValue() * 9.81 * 1.0);
      
      swingTime.set(200.2);

      YoDouble timeProvider = simple3dofBiped.getYoTime();
      stateMachine = new StateMachine<LeapOfFaithState>("state", "switchTime", LeapOfFaithState.class, timeProvider, registry);

      LoadingToeOffState rightLoadingLeftToeOffState = new LoadingToeOffState(LeapOfFaithState.RightLoadingLeftToeOff, LeapOfFaithState.RightSupportLeftSwing);
      stateMachine.addState(rightLoadingLeftToeOffState);
      
      SupportSwingState rightSupportLeftSwingState = new SupportSwingState(LeapOfFaithState.RightSupportLeftSwing, LeapOfFaithState.RightDropLeftRetract);
      stateMachine.addState(rightSupportLeftSwingState);

      stateMachine.setCurrentState(LeapOfFaithState.RightLoadingLeftToeOff);
   }

   private class LoadingToeOffState extends SimpleState<LeapOfFaithState>
   {
      private RobotSide loadingSide, toeOffSide;

      public LoadingToeOffState(LeapOfFaithState stateEnum, LeapOfFaithState nextStateEnum)
      {
         super(stateEnum, nextStateEnum);

         loadingSide = stateEnum.getLoadingSide();
         toeOffSide = stateEnum.getToeOffSide();
      }

      @Override
      public void doTransitionIntoAction()
      {
         kneePDControllers.get(loadingSide).setProportionalGain(2000.0);
         kneePDControllers.get(loadingSide).setDerivativeGain(200.0);

         kneePDControllers.get(toeOffSide).setProportionalGain(200.0);
         kneePDControllers.get(toeOffSide).setDerivativeGain(20.0);

         q_d_knees.get(loadingSide).set(robot.getKneeLength(loadingSide));
         q_d_knees.get(toeOffSide).set(robot.getKneeLength(toeOffSide));

         super.doTransitionIntoAction();
      }

      @Override
      public void doAction()
      {
         computeKneeForce(RobotSide.LEFT);
         computeKneeForce(RobotSide.RIGHT);

         q_d_knees.get(toeOffSide).add(deltaTime.getDoubleValue() * toeOffKneeVelocity.getDoubleValue());

         if (robot.getForwardVelocity() > 0.5)
         {
            super.transitionToDefaultNextState();
         }
      }
   };

   private class SupportSwingState extends SimpleState<LeapOfFaithState>
   {
      private RobotSide supportSide, swingSide;

      public SupportSwingState(LeapOfFaithState stateEnum, LeapOfFaithState nextStateEnum)
      {
         super(stateEnum, nextStateEnum);

         supportSide = stateEnum.getSupportSide();
         swingSide = stateEnum.getSwingSide();
      }

      @Override
      public void doTransitionIntoAction()
      {
         kneePDControllers.get(supportSide).setProportionalGain(2000.0);
         kneePDControllers.get(supportSide).setDerivativeGain(200.0);

         kneePDControllers.get(swingSide).setProportionalGain(200.0);
         kneePDControllers.get(swingSide).setDerivativeGain(20.0);

         q_d_knees.get(supportSide).set(robot.getKneeLength(supportSide));
         q_d_knees.get(swingSide).set(robot.getKneeLength(swingSide));

         super.doTransitionIntoAction();
      }

      @Override
      public void doAction()
      {
         computeKneeForce(RobotSide.LEFT);
         computeKneeForce(RobotSide.RIGHT);

         q_d_knees.get(swingSide).sub(deltaTime.getDoubleValue() * retractKneeForSwingVelocity.getDoubleValue());
         if (q_d_knees.get(swingSide).getDoubleValue() < 0.9)
         {
            q_d_knees.get(swingSide).set(0.9);
         }

         if (stateMachine.timeInCurrentState() > swingTime.getDoubleValue())
         {
            super.transitionToDefaultNextState();
         }
      }
   };

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      deltaTime.set(robot.getTime() - previousTickTime.getDoubleValue());
      if (deltaTime.getDoubleValue() < 0.0)
      {
         deltaTime.set(0.0);
      }
      previousTickTime.set(robot.getTime());

      robot.computeCapturePoint();

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
   }

   private void computeKneeForce(RobotSide robotSide)
   {
      double kneeLength = robot.getKneeLength(robotSide);
      double kneeVelocity = robot.getKneeVelocity(robotSide);

      double desiredKneePosition = q_d_knees.get(robotSide).getDoubleValue();
      double desiredKneeVelocity = 0.0;

      double kneeForce = kneePDControllers.get(robotSide).computeForAngles(kneeLength, desiredKneePosition, kneeVelocity, desiredKneeVelocity);

      if (kneeForce < minimumKneeForce.getDoubleValue())
      {
         kneeForce = minimumKneeForce.getDoubleValue();
      }

      if (kneeForce > maximumKneeForce.getDoubleValue())
      {
         kneeForce = maximumKneeForce.getDoubleValue();
      }

      robot.setKneeForce(robotSide, kneeForce);
   }

   public enum LeapOfFaithState
   {
      RightLoadingLeftToeOff, RightSupportLeftSwing, RightDropLeftRetract, LeftLoadingRightToeOff, LeftSupportRightSwing, LeftDropRightRetract;

      public RobotSide getLoadingSide()
      {
         if (this == LeftLoadingRightToeOff)
            return RobotSide.LEFT;
         if (this == RightLoadingLeftToeOff)
            return RobotSide.RIGHT;
         throw new RuntimeException("No Loading in this state!");
      }

      public RobotSide getToeOffSide()
      {
         return getLoadingSide().getOppositeSide();
      }

      public RobotSide getSwingSide()
      {
         if (this == RightSupportLeftSwing)
            return RobotSide.LEFT;
         if (this == LeftSupportRightSwing)
            return RobotSide.RIGHT;
         throw new RuntimeException("No Swing in this state!");
      }

      public RobotSide getSupportSide()
      {
         return getSwingSide().getOppositeSide();
      }

   }

}
