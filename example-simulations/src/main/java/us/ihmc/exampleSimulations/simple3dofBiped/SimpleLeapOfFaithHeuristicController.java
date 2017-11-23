package us.ihmc.exampleSimulations.simple3dofBiped;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.trajectories.CubicPolynomialTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
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

   private final YoDouble capturePointToStartSwing = new YoDouble("capturePointToStartSwing", registry);

   private final SideDependentList<YoDouble> thighAngles = new SideDependentList<>();
   private final SideDependentList<YoDouble> thighVelocities = new SideDependentList<>();

   private final SideDependentList<YoDouble> q_d_hips = new SideDependentList<>();
   private final SideDependentList<YoDouble> q_d_knees = new SideDependentList<>();

   
   private final YoDouble q_d_pitch = new YoDouble("q_d_pitch", registry);
   private final YoDouble bodyOrientationKp = new YoDouble("bodyOrientationKp", registry);
   private final YoDouble bodyOrientationKd = new YoDouble("bodyOrientationKd", registry);

   private final YoDouble kneeSupportKp = new YoDouble("kneeSupportKp", registry);
   private final YoDouble kneeSupportKd = new YoDouble("kneeSupportKd", registry);

   private final YoDouble kneeSwingKp = new YoDouble("kneeSwingKp", registry);
   private final YoDouble kneeSwingKd = new YoDouble("kneeSwingKd", registry);

   private final YoDouble minimumKneeSupportForce = new YoDouble("minimumKneeSupportForce", registry);

   private final YoDouble thighSwingKp = new YoDouble("thighSwingKp", registry);
   private final YoDouble thighSwingKd = new YoDouble("thighSwingKd", registry);

   private final YoDouble previousTickTime = new YoDouble("previousTickTime", registry);
   private final YoDouble deltaTime = new YoDouble("deltaTime", registry);
   private final YoDouble maxToeOffKneeLength = new YoDouble("maxToeOffKneeLength", registry);

   private final YoDouble totalMass = new YoDouble("totalMass", registry);
   private final YoDouble minimumKneeForce = new YoDouble("minimumKneeForce", registry);
   private final YoDouble maximumKneeForce = new YoDouble("maximumKneeForce", registry);

   private final YoDouble toeOffKneeAcceleration = new YoDouble("toeOffKneeAcceleration", registry);
   private final YoDouble toeOffKneeVelocity = new YoDouble("toeOffKneeVelocity", registry);

   private final YoDouble dropSupportKneeAcceleration = new YoDouble("dropSupportKneeAcceleration", registry);
   private final YoDouble dropSupportKneeVelocity = new YoDouble("dropSupportKneeVelocity", registry);

   private final YoVariableDoubleProvider finalSwingThighAngle = new YoVariableDoubleProvider("finalSwingThighAngle", registry);
   private final YoVariableDoubleProvider swingDuration = new YoVariableDoubleProvider("swingDuration", registry);

   private final YoVariableDoubleProvider finalRetractThighAngle = new YoVariableDoubleProvider("finalRetractThighAngle", registry);
   private final YoVariableDoubleProvider retractDuration = new YoVariableDoubleProvider("retractDuration", registry);

   private final YoDouble swingTime = new YoDouble("swingTime", registry);
   private final YoDouble retractKneeForSwingVelocity = new YoDouble("retractKneeForSwingVelocity", registry);
   private final YoDouble straightenKneeForSwingVelocity = new YoDouble("straightenKneeForSwingVelocity", registry);

   private final YoDouble extendKneeDuringStanceVelocity = new YoDouble("extendKneeDuringStanceVelocity", registry);

   private final YoDouble minimumSupportKneeLength = new YoDouble("minimumSupportKneeLength", registry);
   private final YoDouble maximumSupportKneeLength = new YoDouble("maximumSupportKneeLength", registry);

   private final StateMachine<LeapOfFaithState> stateMachine;

   public SimpleLeapOfFaithHeuristicController(Simple3dofBipedRobot simple3dofBiped)
   {
      this.robot = simple3dofBiped;

      for (RobotSide robotSide : RobotSide.values)
      {
         YoDouble thighAngle = new YoDouble(robotSide.getCamelCaseName() + "ThighAngle", registry);
         YoDouble thighVelocity = new YoDouble(robotSide.getCamelCaseName() + "ThighVelocity", registry);

         YoDouble q_d_hip = new YoDouble("q_d_" + robotSide.getCamelCaseNameForMiddleOfExpression() + "Hip", registry);
         YoDouble q_d_knee = new YoDouble("q_d_" + robotSide.getCamelCaseNameForMiddleOfExpression() + "Knee", registry);

         thighAngles.set(robotSide, thighAngle);
         thighVelocities.set(robotSide, thighVelocity);

         q_d_hips.set(robotSide, q_d_hip);
         q_d_knees.set(robotSide, q_d_knee);
      }

      Point3D comPoint = new Point3D();
      totalMass.set(simple3dofBiped.computeCenterOfMass(comPoint));

      finalSwingThighAngle.set(-0.4);
      swingDuration.set(0.3);
      swingTime.set(0.3);
      finalRetractThighAngle.set(-0.2);
      retractDuration.set(0.3);

      toeOffKneeAcceleration.set(4.0);
      retractKneeForSwingVelocity.set(5.0);
      straightenKneeForSwingVelocity.set(3.0);
      extendKneeDuringStanceVelocity.set(0.2);
      dropSupportKneeAcceleration.set(4.0);//-2

      capturePointToStartSwing.set(0.10);

      minimumSupportKneeLength.set(0.8);
      maximumSupportKneeLength.set(1.0);
      maxToeOffKneeLength.set(1.1);

      bodyOrientationKp.set(50.0);
      bodyOrientationKd.set(5.0);

      kneeSupportKp.set(500.0);
      kneeSupportKd.set(50.0);

      kneeSwingKp.set(500.0);
      kneeSwingKd.set(10.0);

      thighSwingKp.set(40.0);
      thighSwingKd.set(1.0);

      minimumKneeSupportForce.set(2.0);

      minimumKneeForce.set(-totalMass.getDoubleValue() * 9.81 * 0.2);
      maximumKneeForce.set(totalMass.getDoubleValue() * 9.81 * 1.1);

      YoDouble timeProvider = simple3dofBiped.getYoTime();
      stateMachine = new StateMachine<LeapOfFaithState>("state", "switchTime", LeapOfFaithState.class, timeProvider, registry);

      LoadingToeOffState rightLoadingLeftToeOffState = new LoadingToeOffState(LeapOfFaithState.RightLoadingLeftToeOff, LeapOfFaithState.RightSupportLeftSwing);
      stateMachine.addState(rightLoadingLeftToeOffState);

      SupportSwingState rightSupportLeftSwingState = new SupportSwingState(LeapOfFaithState.RightSupportLeftSwing, LeapOfFaithState.RightDropLeftRetract,
                                                                           registry);
      stateMachine.addState(rightSupportLeftSwingState);

      DropRetractState rightDropLeftRetractState = new DropRetractState(LeapOfFaithState.RightDropLeftRetract, LeapOfFaithState.LeftLoadingRightToeOff,
                                                                        registry);
      stateMachine.addState(rightDropLeftRetractState);

      LoadingToeOffState leftLoadingRightToeOffState = new LoadingToeOffState(LeapOfFaithState.LeftLoadingRightToeOff, LeapOfFaithState.LeftSupportRightSwing);
      stateMachine.addState(leftLoadingRightToeOffState);

      SupportSwingState leftSupportRightSwingState = new SupportSwingState(LeapOfFaithState.LeftSupportRightSwing, LeapOfFaithState.LeftDropRightRetract,
                                                                           registry);
      stateMachine.addState(leftSupportRightSwingState);

      DropRetractState leftDropRightRetractState = new DropRetractState(LeapOfFaithState.LeftDropRightRetract, LeapOfFaithState.RightLoadingLeftToeOff,
                                                                        registry);
      stateMachine.addState(leftDropRightRetractState);

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
         q_d_knees.get(loadingSide).set(robot.getKneeLength(loadingSide));
         q_d_knees.get(toeOffSide).set(robot.getKneeLength(toeOffSide));

         toeOffKneeVelocity.set(0.0);
         super.doTransitionIntoAction();
      }

      @Override
      public void doAction()
      {
         double supportHipTorque = bodyOrientationKp.getDoubleValue() * (q_d_pitch.getDoubleValue() - robot.getBodyAngle())
               - bodyOrientationKd.getDoubleValue() * robot.getBodyAngularVelocity();
         Vector3D loadingGroundForce = robot.getFootForce(loadingSide);
         Vector3D toeOffGroundForce = robot.getFootForce(toeOffSide);
         
         double loadingMagnitude = loadingGroundForce.getZ();
         double toeOffMagnitude = toeOffGroundForce.getZ();
         
         if (loadingMagnitude < 0.0) loadingMagnitude = 0.0;
         if (toeOffMagnitude < 0.0) toeOffMagnitude = 0.0;
         double totalMagnitude = loadingMagnitude + toeOffMagnitude;
         
         if (totalMagnitude > totalMass.getDoubleValue() * 9.81 * 0.05) 
         {
            double percentLoading = loadingMagnitude / totalMagnitude;
            double percentToeOff = toeOffMagnitude / totalMagnitude;
            
            robot.setHipTorque(loadingSide, -supportHipTorque * percentLoading);
            robot.setHipTorque(toeOffSide, -supportHipTorque * percentToeOff);
         }
         
         YoDouble toeOffKneeDesiredLength = q_d_knees.get(toeOffSide);

         toeOffKneeVelocity.add(deltaTime.getDoubleValue() * toeOffKneeAcceleration.getDoubleValue());
         toeOffKneeDesiredLength.add(deltaTime.getDoubleValue() * toeOffKneeVelocity.getDoubleValue());

         if (toeOffKneeDesiredLength.getDoubleValue() > maxToeOffKneeLength.getDoubleValue())
         {
            toeOffKneeDesiredLength.set(maxToeOffKneeLength.getDoubleValue());
         }

         double loadingKneeForce = kneeSupportKp.getDoubleValue() * (q_d_knees.get(loadingSide).getDoubleValue() - robot.getKneeLength(loadingSide))
               - kneeSupportKd.getDoubleValue() * robot.getKneeVelocity(loadingSide);
         robot.setKneeForce(loadingSide, loadingKneeForce);

         if (loadingKneeForce < minimumKneeForce.getDoubleValue())
         {
            loadingKneeForce = minimumKneeForce.getDoubleValue();
         }

         double toeOffKneeForce = kneeSupportKp.getDoubleValue() * (q_d_knees.get(toeOffSide).getDoubleValue() - robot.getKneeLength(toeOffSide))
               - kneeSupportKd.getDoubleValue() * robot.getKneeVelocity(toeOffSide);

         if (toeOffKneeForce < minimumKneeSupportForce.getDoubleValue())
         {
            toeOffKneeForce = minimumKneeSupportForce.getDoubleValue();
            if (q_d_knees.get(toeOffSide).getDoubleValue() < robot.getKneeLength(toeOffSide))
            {
               q_d_knees.get(toeOffSide).set(robot.getKneeLength(toeOffSide));
            }
         }

         robot.setKneeForce(toeOffSide, toeOffKneeForce);

         if (robot.getCapturePointXWithRespectToFoot(loadingSide) > capturePointToStartSwing.getDoubleValue())
         {
            super.transitionToDefaultNextState();
         }
      }
   };

   private class SupportSwingState extends SimpleState<LeapOfFaithState>
   {
      private RobotSide supportSide, swingSide;
      private final CubicPolynomialTrajectoryGenerator swingTrajectory;
      private final YoVariableDoubleProvider initialSwingThighAngle;
      private final YoDouble desiredSwingThighAngle;

      public SupportSwingState(LeapOfFaithState stateEnum, LeapOfFaithState nextStateEnum, YoVariableRegistry parentRegistry)
      {
         super(stateEnum, nextStateEnum);

         supportSide = stateEnum.getSupportSide();
         swingSide = stateEnum.getSwingSide();

         String swingSideName = swingSide.getCamelCaseName();

         initialSwingThighAngle = new YoVariableDoubleProvider(swingSideName + "InitialSwingThighAngle", parentRegistry);

         swingTrajectory = new CubicPolynomialTrajectoryGenerator(swingSideName + "Swing", initialSwingThighAngle, finalSwingThighAngle, swingDuration,
                                                                  parentRegistry);

         desiredSwingThighAngle = new YoDouble(swingSideName + "DesiredSwingThighAngle", parentRegistry);
      }

      @Override
      public void doTransitionIntoAction()
      {
         initialSwingThighAngle.set(thighAngles.get(swingSide).getDoubleValue());
         swingTrajectory.initialize();

         q_d_knees.get(swingSide).set(robot.getKneeLength(swingSide));

         super.doTransitionIntoAction();
      }

      @Override
      public void doAction()
      {
         double supportHipTorque = bodyOrientationKp.getDoubleValue() * (q_d_pitch.getDoubleValue() - robot.getBodyAngle())
               - bodyOrientationKd.getDoubleValue() * robot.getBodyAngularVelocity();
         robot.setHipTorque(supportSide, -supportHipTorque);

         swingTrajectory.compute(getTimeInCurrentState());
         desiredSwingThighAngle.set(swingTrajectory.getValue());

         double swingHipTorque = thighSwingKp.getDoubleValue() * (desiredSwingThighAngle.getDoubleValue() - thighAngles.get(swingSide).getDoubleValue())
               - thighSwingKd.getDoubleValue() * thighVelocities.get(swingSide).getDoubleValue();
         robot.setHipTorque(swingSide, swingHipTorque);

         q_d_knees.get(supportSide).add(deltaTime.getDoubleValue() * extendKneeDuringStanceVelocity.getDoubleValue());
         if (q_d_knees.get(supportSide).getDoubleValue() > 1.0)
         {
            q_d_knees.get(supportSide).set(1.0);
         }

         q_d_knees.get(swingSide).sub(deltaTime.getDoubleValue() * retractKneeForSwingVelocity.getDoubleValue());
         if (q_d_knees.get(swingSide).getDoubleValue() < 0.8)
         {
            q_d_knees.get(swingSide).set(0.8);
         }

         double supportKneeForce = kneeSupportKp.getDoubleValue() * (q_d_knees.get(supportSide).getDoubleValue() - robot.getKneeLength(supportSide))
               - kneeSupportKd.getDoubleValue() * robot.getKneeVelocity(supportSide);
         if (supportKneeForce < minimumKneeSupportForce.getDoubleValue())
         {
            supportKneeForce = minimumKneeSupportForce.getDoubleValue();
         }
         robot.setKneeForce(supportSide, supportKneeForce);

         double swingKneeForce = kneeSwingKp.getDoubleValue() * (q_d_knees.get(swingSide).getDoubleValue() - robot.getKneeLength(swingSide))
               - kneeSwingKd.getDoubleValue() * robot.getKneeVelocity(swingSide);
         robot.setKneeForce(swingSide, swingKneeForce);

         if (stateMachine.timeInCurrentState() > swingTime.getDoubleValue())
         {
            super.transitionToDefaultNextState();
         }
      }
   };

   private class DropRetractState extends SimpleState<LeapOfFaithState>
   {
      private RobotSide supportSide, swingSide;
      private final CubicPolynomialTrajectoryGenerator retractTrajectory;
      private final YoVariableDoubleProvider initialRetractThighAngle;
      private final YoDouble desiredRetractThighAngle;

      public DropRetractState(LeapOfFaithState stateEnum, LeapOfFaithState nextStateEnum, YoVariableRegistry parentRegistry)
      {
         super(stateEnum, nextStateEnum);

         supportSide = stateEnum.getSupportSide();
         swingSide = stateEnum.getSwingSide();

         String swingSideName = swingSide.getCamelCaseName();

         initialRetractThighAngle = new YoVariableDoubleProvider(swingSideName + "InitialRetractThighAngle", parentRegistry);

         retractTrajectory = new CubicPolynomialTrajectoryGenerator(swingSideName + "Retract", initialRetractThighAngle, finalRetractThighAngle,
                                                                    retractDuration, parentRegistry);

         desiredRetractThighAngle = new YoDouble(swingSideName + "DesiredRetractThighAngle", parentRegistry);
      }

      @Override
      public void doTransitionIntoAction()
      {
         initialRetractThighAngle.set(thighAngles.get(swingSide).getDoubleValue());
         retractTrajectory.initialize();

         q_d_knees.get(swingSide).set(robot.getKneeLength(swingSide));
         dropSupportKneeVelocity.set(0.0);

         super.doTransitionIntoAction();
      }

      @Override
      public void doAction()
      {
         double supportHipTorque = bodyOrientationKp.getDoubleValue() * (q_d_pitch.getDoubleValue() - robot.getBodyAngle())
               - bodyOrientationKd.getDoubleValue() * robot.getBodyAngularVelocity();
         robot.setHipTorque(supportSide, -supportHipTorque);

         retractTrajectory.compute(getTimeInCurrentState());
         desiredRetractThighAngle.set(retractTrajectory.getValue());

         double swingHipTorque = thighSwingKp.getDoubleValue() * (desiredRetractThighAngle.getDoubleValue() - thighAngles.get(swingSide).getDoubleValue())
               - thighSwingKd.getDoubleValue() * thighVelocities.get(swingSide).getDoubleValue();
         robot.setHipTorque(swingSide, swingHipTorque);

         q_d_knees.get(swingSide).add(deltaTime.getDoubleValue() * straightenKneeForSwingVelocity.getDoubleValue());
         if (q_d_knees.get(swingSide).getDoubleValue() > 1.0)
         {
            q_d_knees.get(swingSide).set(1.0);
         }

         dropSupportKneeVelocity.add(deltaTime.getDoubleValue() * dropSupportKneeAcceleration.getDoubleValue());
         q_d_knees.get(supportSide).sub(deltaTime.getDoubleValue() * dropSupportKneeVelocity.getDoubleValue());
         if (q_d_knees.get(supportSide).getDoubleValue() < minimumSupportKneeLength.getDoubleValue())
         {
            q_d_knees.get(supportSide).set(minimumSupportKneeLength.getDoubleValue());
         }
         if (q_d_knees.get(supportSide).getDoubleValue() > maximumSupportKneeLength.getDoubleValue())
         {
            q_d_knees.get(supportSide).set(maximumSupportKneeLength.getDoubleValue());
         }

         double supportKneeForce = kneeSupportKp.getDoubleValue() * (q_d_knees.get(supportSide).getDoubleValue() - robot.getKneeLength(supportSide))
               - kneeSupportKd.getDoubleValue() * robot.getKneeVelocity(supportSide);
         if (supportKneeForce < minimumKneeSupportForce.getDoubleValue())
         {
            supportKneeForce = minimumKneeSupportForce.getDoubleValue();
         }
         robot.setKneeForce(supportSide, supportKneeForce);

         double swingKneeForce = kneeSwingKp.getDoubleValue() * (q_d_knees.get(swingSide).getDoubleValue() - robot.getKneeLength(swingSide))
               - kneeSwingKd.getDoubleValue() * robot.getKneeVelocity(swingSide);
         robot.setKneeForce(swingSide, swingKneeForce);

         if (robot.hasFootMadeContact(swingSide))
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

      for (RobotSide robotSide : RobotSide.values)
      {
         thighAngles.get(robotSide).set(robot.getThighAngle(robotSide));
         thighVelocities.get(robotSide).set(robot.getThighAngularVelocity(robotSide));
      }

      robot.computeCapturePoint();

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
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
         if (this == RightDropLeftRetract)
            return RobotSide.LEFT;
         if (this == LeftDropRightRetract)
            return RobotSide.RIGHT;
         throw new RuntimeException("No Swing in this state! " + this);
      }

      public RobotSide getSupportSide()
      {
         return getSwingSide().getOppositeSide();
      }

   }

}
