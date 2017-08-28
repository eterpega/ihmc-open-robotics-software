package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.exampleSimulations.planarWalker.PlanarWalkerController.ControllerState;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.math.trajectories.YoMinimumJerkTrajectory;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class Test
{
   public enum ControllerState
   {
      A, B, C;
   }

   private YoVariableRegistry registry = new YoVariableRegistry("Controller");

   private YoMinimumJerkTrajectory trajectorySwingHip;
   private YoMinimumJerkTrajectory trajectorySwingKnee;

   private PIDController A;
   private PIDController B;

   private StateMachine<ControllerState> stateMachine;
   private YoDouble timeInState = new YoDouble("timeInState", registry);

   Test()
   {
      A = new PIDController("controllerA", registry);
      B = new PIDController("controllerB", registry);
      trajectorySwingHip = new YoMinimumJerkTrajectory("trajectorySwingHip", registry);
      trajectorySwingKnee = new YoMinimumJerkTrajectory("trajectorySwingKnee", registry);

      A.setProportionalGain(100.0);
      A.setDerivativeGain(20.0);
      B.setProportionalGain(100.0);
      B.setDerivativeGain(20.0);

      stateMachine = new StateMachine<ControllerState>("controllerState", "switchTime", ControllerState.class, new YoDouble("time", registry), registry);

      A aState = new A(ControllerState.A);
      B bState = new B(ControllerState.B);

      aState.setDefaultNextState(bState.getStateEnum());
      bState.setDefaultNextState(aState.getStateEnum());

      stateMachine.addState(aState);
      stateMachine.addState(bState);

      stateMachine.setCurrentState(aState.getStateEnum());

   }

   private void planTrajectory()
   {
      trajectorySwingKnee.setParams(0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 2.0);

      for (float i = 0; i < 2.0; i = (float) (i + 0.1))
      {
         trajectorySwingKnee.computeTrajectory(i);
         System.out.println("time " + i);
         System.out.println("position " + trajectorySwingKnee.getPosition());
         System.out.println("velocity " + trajectorySwingKnee.getVelocity());
      }

   }

   public static void main(String[] args)
   {
      Test t = new Test();
      //      t.planTrajectory();
      while (true)
      {
         t.stateMachine.doAction();
         t.stateMachine.checkTransitionConditions();
      }
   }

   private class A extends State<ControllerState>
   {

      public A(ControllerState stateEnum)
      {
         super(stateEnum);
      }

      @Override
      public void doAction()
      {
         System.out.println("time in state A: " + this.getTimeInCurrentState());
         System.out.println("state A");
         this.transitionToDefaultNextState();
      }

      @Override
      public void doTransitionIntoAction()
      {
         System.out.println("transit into A");
      }

      @Override
      public void doTransitionOutOfAction()
      {
         System.out.println("transit out of A");
      }
   }

   private class B extends State<ControllerState>
   {

      public B(ControllerState stateEnum)
      {
         super(stateEnum);
      }

      @Override
      public void doAction()
      {
         System.out.println("time in state B: " + this.getTimeInCurrentState());
         System.out.println("state B");
         if(this.getTimeInCurrentState() > 0.5)
         this.transitionToDefaultNextState();
      }

      @Override
      public void doTransitionIntoAction()
      {
         System.out.println("transit into B");
      }

      @Override
      public void doTransitionOutOfAction()
      {
         System.out.println("transit out of B");
      }
   }

}
