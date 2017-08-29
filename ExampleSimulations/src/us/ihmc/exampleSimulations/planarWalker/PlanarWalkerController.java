package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.math.trajectories.YoMinimumJerkTrajectory;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

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
	private YoMinimumJerkTrajectory trajectoryStanceHip;
	private YoMinimumJerkTrajectory trajectoryStanceKnee;

	private double HIP_DEFUALT_P_GAIN = 100.0;
	private double HIP_DEFUALT_D_GAIN = 10.0;

	private double KNEE_DEFUALT_P_GAIN = 10000.0;
	private double KNEE_DEFUALT_D_GAIN = 1000.0;

	private double SWING_TIME = 2.0;
	public static double START_TIME;
	private static boolean GOTO_NEXT_STATE = false;
	private static double PITCH_ANGLE;
	private static double LEG_LENGTH ;
	private static double STEP_LENGTH = 0.3;
	private static double KNEE_EXTENSION = 0.4;
	private static double KNEE_RETRACTION = 0.2;

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

		trajectoryStanceHip = new YoMinimumJerkTrajectory("trajectoryStanceHip", registry);
		trajectoryStanceKnee = new YoMinimumJerkTrajectory("trajectoryStanceKnee", registry);

		LEG_LENGTH = robot.nominalHeight;

		initialize();
	}

	public PlanarWalkerController(PeterPlanarWalkerRobot robot, double deltaT, boolean runInDebug)
	{
		this(robot,deltaT);
		this.runInDebug = runInDebug;
	}

	private double calcPitchAngle() {
		return Math.asin(STEP_LENGTH/LEG_LENGTH);
	}

	public enum ControllerState
	{
		START, LEFT_STEP, RIGHT_STEP;
	}

	@Override
	public void initialize()
	{
		// initialize state machine 
		stateMachine = new StateMachine<ControllerState>("controllerState", "switchTime", ControllerState.class, robot.getYoTime(),
				registry);

		StartState startState = new StartState(ControllerState.START);

		WalkState leftStepState = new WalkState(ControllerState.LEFT_STEP);
		WalkState rightStepState = new WalkState(ControllerState.RIGHT_STEP);

		startState.setDefaultNextState(rightStepState.getStateEnum());
		rightStepState.setDefaultNextState(leftStepState.getStateEnum());
		leftStepState.setDefaultNextState(rightStepState.getStateEnum());

		stateMachine.addState(startState);
		stateMachine.addState(leftStepState);
		stateMachine.addState(rightStepState);

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
			// get into the starting position



			if(this.getTimeInCurrentState() - START_TIME < SWING_TIME/2.0)
			{

				// swing leg
				trajectorySwingHip.setParams(robot.getHipPosition(RobotSide.RIGHT), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, SWING_TIME/2.0);
				trajectorySwingKnee.setParams(robot.getKneePosition(RobotSide.RIGHT), 0.0, 0.0, KNEE_RETRACTION, 0.0, 0.0, 0.0, SWING_TIME/2.0);
				trajectorySwingHip.computeTrajectory(this.getTimeInCurrentState() - START_TIME);
				trajectorySwingKnee.computeTrajectory(this.getTimeInCurrentState() - START_TIME);

				// stance leg
				trajectoryStanceHip.setParams(robot.getHipPosition(RobotSide.LEFT), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, SWING_TIME/2.0);
				trajectoryStanceKnee.setParams(robot.getKneePosition(RobotSide.LEFT), 0.0, 0.0, KNEE_EXTENSION, 0.0, 0.0, 0.0, SWING_TIME/2.0);
				trajectoryStanceHip.computeTrajectory(this.getTimeInCurrentState() - START_TIME);
				trajectoryStanceKnee.computeTrajectory(this.getTimeInCurrentState() - START_TIME);

				sendTorques(RobotSide.RIGHT, 
						trajectorySwingKnee.getPosition(), trajectorySwingKnee.getVelocity(), 
						trajectorySwingHip.getPosition(), trajectorySwingHip.getVelocity(),
						trajectoryStanceKnee.getPosition(), trajectoryStanceKnee.getVelocity(), 
						trajectoryStanceHip.getPosition(), trajectoryStanceHip.getVelocity());
			}
			else if(this.getTimeInCurrentState() - START_TIME > SWING_TIME/2.0 && this.getTimeInCurrentState() - START_TIME < SWING_TIME)
			{

				// swing leg
				trajectorySwingHip.setParams(robot.getHipPosition(RobotSide.RIGHT), 0.0, 0.0, 0.0, PITCH_ANGLE, 0.0, 0.0, SWING_TIME/2.0);
				trajectorySwingKnee.setParams(robot.getKneePosition(RobotSide.RIGHT), 0.0, 0.0, KNEE_RETRACTION, 0.0, 0.0, 0.0, SWING_TIME/2.0);
				trajectorySwingHip.computeTrajectory(this.getTimeInCurrentState() - START_TIME - SWING_TIME/2.0);
				trajectorySwingKnee.computeTrajectory(this.getTimeInCurrentState() - START_TIME - SWING_TIME/2.0);

				// stance leg
				trajectoryStanceHip.setParams(robot.getHipPosition(RobotSide.LEFT), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, SWING_TIME/2.0);
				trajectoryStanceKnee.setParams(robot.getKneePosition(RobotSide.LEFT), 0.0, 0.0, KNEE_EXTENSION, 0.0, 0.0, 0.0, SWING_TIME/2.0);
				trajectoryStanceHip.computeTrajectory(this.getTimeInCurrentState() - START_TIME - SWING_TIME/2.0);
				trajectoryStanceKnee.computeTrajectory(this.getTimeInCurrentState() - START_TIME - SWING_TIME/2.0);

				sendTorques(RobotSide.RIGHT, 
						trajectorySwingKnee.getPosition(), trajectorySwingKnee.getVelocity(), 
						trajectorySwingHip.getPosition(), trajectorySwingHip.getVelocity(),
						trajectoryStanceKnee.getPosition(), trajectoryStanceKnee.getVelocity(), 
						trajectoryStanceHip.getPosition(), trajectoryStanceHip.getVelocity());
			}
			else GOTO_NEXT_STATE = true;

			if(GOTO_NEXT_STATE == true)
			{
				if(runInDebug) System.out.println("transit from start state to next state");

				this.transitionToDefaultNextState();
			}
		}

		@Override
		public void doTransitionIntoAction()
		{
			// calc pitch angle

			PITCH_ANGLE = calcPitchAngle();
			System.out.println("pitch angle "+PITCH_ANGLE);

			GOTO_NEXT_STATE = false;
			START_TIME = this.getTimeInCurrentState();
		}

		@Override
		public void doTransitionOutOfAction(){}  
	}

	/**
	 * State Machine class for the half steps tate
	 */
	private class WalkState extends State<ControllerState>
	{
		RobotSide legSide ;
		public WalkState(ControllerState stateEnum)
		{
			super(stateEnum);
			if(stateEnum.equals(ControllerState.LEFT_STEP))
				legSide = RobotSide.LEFT;
			else if(stateEnum.equals(ControllerState.RIGHT_STEP))
				legSide = RobotSide.RIGHT;
			else throw new StateMismatchException("State assignment Incorrect"); // to check if state assignment is incorrect
		}

		@Override
		public void doAction()
		{
			if(this.getTimeInCurrentState() - START_TIME < SWING_TIME/3.0)
			{

				// swing leg
				trajectorySwingHip.setParams(robot.getHipPosition(legSide), 0.0, 0.0, PITCH_ANGLE, 0.0, 0.0, 0.0, SWING_TIME/3.0);
				trajectorySwingKnee.setParams(robot.getKneePosition(legSide), 0.0, 0.0, KNEE_EXTENSION, 0.0, 0.0, 0.0, SWING_TIME/3.0);
				trajectorySwingHip.computeTrajectory(this.getTimeInCurrentState() - START_TIME);
				trajectorySwingKnee.computeTrajectory(this.getTimeInCurrentState() - START_TIME);

				// stance leg
				trajectoryStanceHip.setParams(robot.getHipPosition(legSide.getOppositeSide()), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, SWING_TIME/3.0);
				trajectoryStanceKnee.setParams(robot.getKneePosition(legSide.getOppositeSide()), 0.0, 0.0, KNEE_EXTENSION, 0.0, 0.0, 0.0, SWING_TIME/3.0);
				trajectoryStanceHip.computeTrajectory(this.getTimeInCurrentState() - START_TIME);
				trajectoryStanceKnee.computeTrajectory(this.getTimeInCurrentState() - START_TIME);

				if(runInDebug)
				{
					System.out.println("time in state half step "+legSide+" phase 1 "+ this.getTimeInCurrentState());
					//					System.out.println("trajectory time scale :"+(this.getTimeInCurrentState() - START_TIME));
					//					System.out.println("Swing knee "+trajectorySwingKnee.getPosition()+" Swing hip"+trajectorySwingHip.getPosition());
					//					System.out.println("Stance knee "+trajectoryStanceKnee.getPosition()+" Stance hip"+trajectoryStanceHip.getPosition());
				}

				sendTorques(legSide, 
						trajectorySwingKnee.getPosition(), trajectorySwingKnee.getVelocity(), 
						trajectorySwingHip.getPosition(), trajectorySwingHip.getVelocity(),
						trajectoryStanceKnee.getPosition(), trajectoryStanceKnee.getVelocity(), 
						trajectoryStanceHip.getPosition(), trajectoryStanceHip.getVelocity());


			}
			else if(this.getTimeInCurrentState() - START_TIME > SWING_TIME/3.0 && this.getTimeInCurrentState() - START_TIME < 2*SWING_TIME/3.0)
			{

				// swing leg
				trajectorySwingHip.setParams(robot.getHipPosition(legSide), 0.0, 0.0, PITCH_ANGLE, 0.0, 0.0, 0.0, SWING_TIME/3.0);
				trajectorySwingKnee.setParams(robot.getKneePosition(legSide), 0.0, 0.0, KNEE_EXTENSION, 0.0, 0.0, 0.0, SWING_TIME/3.0);
				trajectorySwingHip.computeTrajectory(this.getTimeInCurrentState() - START_TIME - SWING_TIME/3.0);
				trajectorySwingKnee.computeTrajectory(this.getTimeInCurrentState() - START_TIME - SWING_TIME/3.0);

				// stance leg
				trajectoryStanceHip.setParams(robot.getHipPosition(legSide.getOppositeSide()), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, SWING_TIME/3.0);
				trajectoryStanceKnee.setParams(robot.getKneePosition(legSide.getOppositeSide()), 0.0, 0.0, LEG_LENGTH*Math.cos(PITCH_ANGLE)-robot.upperLinkLength, 0.0, 0.0, 0.0, SWING_TIME/3.0);
				trajectoryStanceHip.computeTrajectory(this.getTimeInCurrentState() - START_TIME - SWING_TIME/3.0);
				trajectoryStanceKnee.computeTrajectory(this.getTimeInCurrentState() - START_TIME - SWING_TIME/3.0);

				if(runInDebug)
				{
					System.out.println("time in state half step "+legSide+" phase 2 "+ this.getTimeInCurrentState());
					//					System.out.println("trajectory time scale :"+(this.getTimeInCurrentState() - START_TIME));
					//					System.out.println("Swing knee "+trajectorySwingKnee.getPosition()+" Swing hip"+trajectorySwingHip.getPosition());
					//					System.out.println("Stance knee "+trajectoryStanceKnee.getPosition()+" Stance hip"+trajectoryStanceHip.getPosition());
				}

				sendTorques(legSide, 
						trajectorySwingKnee.getPosition(), trajectorySwingKnee.getVelocity(), 
						trajectorySwingHip.getPosition(), trajectorySwingHip.getVelocity(),
						trajectoryStanceKnee.getPosition(), trajectoryStanceKnee.getVelocity(), 
						trajectoryStanceHip.getPosition(), trajectoryStanceHip.getVelocity());				
			}
			else if(this.getTimeInCurrentState() - START_TIME > 2*SWING_TIME/3.0 && this.getTimeInCurrentState() - START_TIME < SWING_TIME)
			{

				// swing leg
				trajectorySwingHip.setParams(robot.getHipPosition(legSide), 0.0, 0.0, PITCH_ANGLE, 0.0, 0.0, 0.0, SWING_TIME/3.0);
				trajectorySwingKnee.setParams(robot.getKneePosition(legSide), 0.0, 0.0, KNEE_EXTENSION, 0.0, 0.0, 0.0, SWING_TIME/3.0);
				trajectorySwingHip.computeTrajectory(this.getTimeInCurrentState() - START_TIME - 2*SWING_TIME/3.0);
				trajectorySwingKnee.computeTrajectory(this.getTimeInCurrentState() - START_TIME - 2*SWING_TIME/3.0);

				// stance leg
				trajectoryStanceHip.setParams(robot.getHipPosition(legSide.getOppositeSide()), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, SWING_TIME/3.0);
				trajectoryStanceKnee.setParams(robot.getKneePosition(legSide.getOppositeSide()), 0.0, 0.0, KNEE_EXTENSION, 0.0, 0.0, 0.0, SWING_TIME/3.0);
				trajectoryStanceHip.computeTrajectory(this.getTimeInCurrentState() - START_TIME - 2*SWING_TIME/3.0);
				trajectoryStanceKnee.computeTrajectory(this.getTimeInCurrentState() - START_TIME - 2*SWING_TIME/3.0);

				if(runInDebug)
				{
					System.out.println("time in state half step "+legSide+" phase 3 "+ this.getTimeInCurrentState());
					//					System.out.println("trajectory time scale :"+(this.getTimeInCurrentState() - START_TIME));
					//					System.out.println("Swing knee "+trajectorySwingKnee.getPosition()+" Swing hip"+trajectorySwingHip.getPosition());
					//					System.out.println("Stance knee "+trajectoryStanceKnee.getPosition()+" Stance hip"+trajectoryStanceHip.getPosition());
				}

				sendTorques(legSide, 
						trajectorySwingKnee.getPosition(), trajectorySwingKnee.getVelocity(), 
						trajectorySwingHip.getPosition(), trajectorySwingHip.getVelocity(),
						trajectoryStanceKnee.getPosition(), trajectoryStanceKnee.getVelocity(), 
						trajectoryStanceHip.getPosition(), trajectoryStanceHip.getVelocity());				
			}
			else if(robot.isFootOnGround(legSide))
			{
				GOTO_NEXT_STATE = true;
				if(runInDebug)
				{
					System.out.println("next state flag called for half step "+legSide);
				}

			}
			else System.out.println("lost in another dimension !");


			if(GOTO_NEXT_STATE == true)
				this.transitionToDefaultNextState();

			//			if(runInDebug) System.out.println("transit from "+legSide+" HalfStepState to next state");

		}

		@Override
		public void doTransitionIntoAction()
		{
			GOTO_NEXT_STATE = false;
			START_TIME = this.getTimeInCurrentState();
		}

		@Override
		public void doTransitionOutOfAction()
		{

		}
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
		//		if(runInDebug) System.out.println("new tick");
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

	private void sendTorques(RobotSide swingLegSide, double swingKneePos, double swingKneeVel, double swingHipPos
			, double swingHipVel, double stanceKneePos, double stanceKneeVel, double stanceHipPos
			, double stanceHipVel)
	{
		controlKneeToPosition(swingLegSide, swingKneePos, swingKneeVel);
		controlHipToPosition(swingLegSide, swingHipPos, swingHipVel);
		controlKneeToPosition(swingLegSide.getOppositeSide(), stanceKneePos, stanceKneeVel);
		controlHipToPosition(swingLegSide.getOppositeSide(), stanceHipPos, stanceHipVel);
	}



}
