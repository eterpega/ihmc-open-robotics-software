package us.ihmc.exampleSimulations.planarWalker;


import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.math.trajectories.YoMinimumJerkTrajectory;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class PlanarWalkerController implements RobotController
{
   private double deltaT;
   private PeterPlanarWalkerRobot robot;
   private YoVariableRegistry registry = new YoVariableRegistry("Controller");
   private SideDependentList<PIDController> kneeControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> hipControllers = new SideDependentList<PIDController>();
   private StateMachine<ControllerState> stateMachine;
   private boolean runInDebug;
   
   private YoMinimumJerkTrajectory trajectorySwingHip;
   private YoMinimumJerkTrajectory trajectorySwingKnee;
   
   private double HIP_DEFUALT_P_GAIN = 100.0;
   private double HIP_DEFUALT_D_GAIN = 10.0;

   private double KNEE_DEFUALT_P_GAIN = 10000.0;
   private double KNEE_DEFUALT_D_GAIN = 1000.0;

   public PlanarWalkerController(PeterPlanarWalkerRobot robot, double deltaT)
   {
      super();
      this.deltaT = deltaT;
      this.robot = robot;     
      
      for (RobotSide robotSide : RobotSide.values)
      {
         PIDController pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_Knee", registry);
         pidController.setProportionalGain(KNEE_DEFUALT_P_GAIN);
         pidController.setDerivativeGain(KNEE_DEFUALT_D_GAIN);
         kneeControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_Hip", registry);
         pidController.setProportionalGain(HIP_DEFUALT_P_GAIN);
         pidController.setDerivativeGain(HIP_DEFUALT_D_GAIN);
         hipControllers.put(robotSide, pidController);
      }

      trajectorySwingHip = new YoMinimumJerkTrajectory("trajectorySwingHip", registry);
      trajectorySwingKnee = new YoMinimumJerkTrajectory("trajectorySwingKnee", registry);
      
      initialize();
   }

   public PlanarWalkerController(PeterPlanarWalkerRobot robot, double deltaT, boolean runInDebug)
   {
      this(robot,deltaT);
      this.runInDebug = runInDebug;
   }

   public enum ControllerState
   {
      START, LEFT_HALF_STEP, LEFT_FULL_STEP, RIGHT_HALF_STEP, RIGHT_FULL_STEP;
   }

   @Override
   public void initialize()
   {
      // initialize state machine 
      stateMachine = new StateMachine<ControllerState>("controllerState", "switchTime", ControllerState.class, robot.getYoTime(),
            registry);

      StartState startState = new StartState(ControllerState.START);

      HalfStepState leftHalfStepState = new HalfStepState(ControllerState.LEFT_HALF_STEP);
      HalfStepState rightHalfStepState = new HalfStepState(ControllerState.RIGHT_HALF_STEP);

      FullStepState leftFullStepState = new FullStepState(ControllerState.LEFT_FULL_STEP);
      FullStepState rightFullStepState = new FullStepState(ControllerState.RIGHT_FULL_STEP);

      startState.setDefaultNextState(rightHalfStepState.getStateEnum());
      rightHalfStepState.setDefaultNextState(leftHalfStepState.getStateEnum());
      leftHalfStepState.setDefaultNextState(rightHalfStepState.getStateEnum());
      
      stateMachine.addState(startState);
      stateMachine.addState(rightHalfStepState);
      stateMachine.addState(leftHalfStepState);
      
      stateMachine.setCurrentState(startState.getStateEnum());

   }

   /**
    * State Machine class for the start state
    */
   private class StartState extends State<ControllerState>
   {

      public StartState(ControllerState stateEnum)
      {
         super(stateEnum);
      }

      @Override
      public void doAction()
      {
         this.transitionToDefaultNextState();
         if(runInDebug) System.out.println("transit from start state to next state");
      }

      @Override
      public void doTransitionIntoAction(){}

      @Override
      public void doTransitionOutOfAction(){}  
   }

   /**
    * State Machine class for the half steps tate
    */
   private class HalfStepState extends State<ControllerState>
   {
      RobotSide legSide ;
      public HalfStepState(ControllerState stateEnum)
      {
         super(stateEnum);
         if(stateEnum.equals(ControllerState.LEFT_HALF_STEP))
            legSide = RobotSide.LEFT;
         else if(stateEnum.equals(ControllerState.RIGHT_HALF_STEP))
               legSide = RobotSide.RIGHT;
         else throw new StateMismatchException("State assignment Incorrect"); // to check if state assignment is incorrect
      }

      @Override
      public void doAction()
      {
         System.out.println("do action for "+legSide+ "  starts");
         trajectorySwingKnee.setParams(robot.getKneePosition(legSide), 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 2.0);
         trajectorySwingKnee.computeTrajectory(2.0);
         System.out.println("controlling "+legSide+" knee");
         controlKneeToPosition(legSide, trajectorySwingKnee.getPosition(), trajectorySwingKnee.getVelocity());
         
         trajectorySwingHip.setParams(robot.getHipPosition(legSide), 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 2.0);
         trajectorySwingHip.computeTrajectory(2.0);
         System.out.println("controlling "+legSide+" hip");
         controlHipToPosition(legSide, trajectorySwingKnee.getPosition(), trajectorySwingKnee.getVelocity());
         
         System.out.println("do action for "+legSide+ "  ends");
         
         this.transitionToDefaultNextState();
         if(runInDebug) System.out.println("transit from "+legSide+" HalfStepState to next state");
         
      }

      @Override
      public void doTransitionIntoAction()
      {
         System.out.println("transit into action for "+legSide+ "  starts");
         trajectorySwingHip.setParams(robot.getHipPosition(legSide), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0);
         trajectorySwingHip.computeTrajectory(3.0);
         System.out.println("controlling "+legSide+" hip");
         controlHipToPosition(legSide, trajectorySwingKnee.getPosition(), trajectorySwingKnee.getVelocity());
         
         trajectorySwingKnee.setParams(robot.getKneePosition(legSide), 0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 3.0);
         trajectorySwingKnee.computeTrajectory(3.0);
         
         System.out.println("controlling "+legSide+" knee");
         controlKneeToPosition(legSide, trajectorySwingKnee.getPosition(), trajectorySwingKnee.getVelocity());
         System.out.println("transit into action for "+legSide+ "  done");
      }

      @Override
      public void doTransitionOutOfAction()
      {
         System.out.println("transit out action for "+legSide+ "  starts");
         trajectorySwingKnee.setParams(robot.getKneePosition(legSide), 0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 2.0);
         trajectorySwingKnee.computeTrajectory(2.0);
         System.out.println("controlling "+legSide+" knee");
         controlKneeToPosition(legSide, trajectorySwingKnee.getPosition(), trajectorySwingKnee.getVelocity());
         
         trajectorySwingKnee.setParams(robot.getKneePosition(legSide.getOppositeSide()), 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.05);
         trajectorySwingKnee.computeTrajectory(0.05);
         System.out.println("controlling "+legSide+" knee opposite leg");
         controlKneeToPosition(legSide.getOppositeSide(), trajectorySwingKnee.getPosition(), trajectorySwingKnee.getVelocity());
         
         trajectorySwingHip.setParams(robot.getHipPosition(legSide), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
         trajectorySwingHip.computeTrajectory(1.0);
         System.out.println("controlling "+legSide+" hip");
         controlHipToPosition(legSide, trajectorySwingKnee.getPosition(), trajectorySwingKnee.getVelocity());
         System.out.println("transit out action for "+legSide+ "  done");
         

      }
   }
   
   /**
    * State Machine class for the full step state
    */
   private class FullStepState extends State<ControllerState>
   {
      private RobotSide legSide;

      public FullStepState(ControllerState stateEnum) 
      {
         super(stateEnum);
         if(stateEnum.equals(ControllerState.LEFT_FULL_STEP))
            legSide = RobotSide.LEFT;
         else if(stateEnum.equals(ControllerState.RIGHT_FULL_STEP))
               legSide = RobotSide.RIGHT;
         else throw new StateMismatchException("State assignment Incorrect"); // to check if state assignment is incorrect
      }

      @Override
      public void doAction()
      {
         this.transitionToDefaultNextState();
         if(runInDebug) System.out.println("transit from "+legSide+" FullStepState to next state");
      }

      @Override
      public void doTransitionIntoAction(){}

      @Override
      public void doTransitionOutOfAction(){}
   }



   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public String getDescription()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void doControl()
   {
      if(runInDebug) System.out.println("new tick");
      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
   }
   
   private static class StateMismatchException extends RuntimeException
   {
      public StateMismatchException(String errorMsg)
      {
         super(errorMsg);
      }
   }
   
   private void controlKneeToPosition(RobotSide robotSide, double desiredPosition, double desiredVelocity)
   {
      double kneePosition = robot.getKneePosition(robotSide);
      double kneePositionRate = robot.getKneeVelocity(robotSide);

      double controlEffort = kneeControllers.get(robotSide).compute(kneePosition, desiredPosition, kneePositionRate, desiredVelocity, deltaT);
      robot.setKneeTorque(robotSide, controlEffort);
   }
   
   private void controlHipToPosition(RobotSide robotSide, double desiredPosition, double desiredVelocity)
   {
      double hipPosition = robot.getHipPosition(robotSide);
      double hipPositionRate = robot.getHipVelocity(robotSide);

      double controlEffort = hipControllers.get(robotSide).compute(hipPosition, desiredPosition, hipPositionRate, desiredVelocity, deltaT);
      robot.setHipTorque(robotSide, controlEffort);
   }



}
