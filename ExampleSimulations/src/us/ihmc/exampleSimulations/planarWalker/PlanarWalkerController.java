package us.ihmc.exampleSimulations.planarWalker;


import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class PlanarWalkerController implements RobotController
{
   private double deltaT;
   private PeterPlanarWalkerRobot robot;
   private YoVariableRegistry registry = new YoVariableRegistry("Controller");
   private StateMachine<ControllerState> stateMachine;
   private boolean runInDebug;

   public PlanarWalkerController(PeterPlanarWalkerRobot robot, double deltaT)
   {
      super();
      this.deltaT = deltaT;
      this.robot = robot;      
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
         this.transitionToDefaultNextState();
         robot.setKneeTorque(legSide, 2);
         robot.setHipTorque(legSide, 2);
         if(runInDebug) System.out.println("transit from "+legSide+" HalfStepState to next state");
         
      }

      @Override
      public void doTransitionIntoAction(){}

      @Override
      public void doTransitionOutOfAction(){}
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
   



}
