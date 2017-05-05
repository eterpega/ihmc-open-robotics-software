package us.ihmc.exampleSimulations.simpleArm;

import javax.vecmath.SingularMatrixException;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.linearDynamicSystems.SplitUpMatrixExponentialStateSpaceSystemDiscretizer;
import us.ihmc.robotics.linearDynamicSystems.StateSpaceSystemDiscretizer;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;

public class SimpleArmEstimator extends SimpleRobotController
{
   private final int nStates = 9;
   private final int nMeasurements = 3;
   private final int nInputs = 0;

   // the robot to read data from
   private final SimpleRobotInputOutputMap robot;

   // the state to display in SCS and for rewinding
   private final YoMatrix yoX = new YoMatrix("x", nStates, 1, registry);
   private final YoMatrix yoP = new YoMatrix("P", nStates, nStates, registry);

   // the state vectors
   // the state is defines as [q1, q2, q3, qd1, qd2, qd3, qdd1, qdd2, qdd3]
   private final DenseMatrix64F xPriori = new DenseMatrix64F(nStates, 1);
   private final DenseMatrix64F xPosteriori = new DenseMatrix64F(nStates, 1);

   // the process
   // x_{k} = A x_{k-1} + B u_{k-1} + w_{k-1}
   // with w being the process noise
   private final DenseMatrix64F A = new DenseMatrix64F(nStates, nStates);
   private final DenseMatrix64F B = new DenseMatrix64F(nStates, nInputs);
   private final DenseMatrix64F u = new DenseMatrix64F(nInputs, 1);

   // the measurement
   // the measurement is [q1_m, q2_m, q3_m]
   // z_{k} = H x_{k} + v_{k}
   // with v being the measurement noise
   private final DenseMatrix64F z = new DenseMatrix64F(nMeasurements, 1);
   private final DenseMatrix64F H = new DenseMatrix64F(nMeasurements, nStates);

   // the noise matrices
   // Q is the process and R is the measurement noise covariance
   // p(w) ~ N(0, Q)
   // p(v) ~ N(0, R)
   // white noise with normal probability
   private final DenseMatrix64F Q = new DenseMatrix64F(nStates, nStates);
   private final DenseMatrix64F R = new DenseMatrix64F(nMeasurements, nMeasurements);

   // the kalman filter matrices
   private final DenseMatrix64F K = new DenseMatrix64F(nStates, nMeasurements);
   private final DenseMatrix64F PPriori = new DenseMatrix64F(nStates, nStates);
   private final DenseMatrix64F PPosteriori = new DenseMatrix64F(nStates, nStates);

   private final StateSpaceSystemDiscretizer discretizer;
   private final LinearSolver<DenseMatrix64F> solver;

   // tuning variables
   private final DoubleYoVariable jointPositionVariance = new DoubleYoVariable("jointPositionVariance", registry);
   private final DoubleYoVariable processJerkVariance = new DoubleYoVariable("processJerkVariance", registry);

   public SimpleArmEstimator(SimpleRobotInputOutputMap robot, double dt)
   {
      this.robot = robot;
      discretizer = new SplitUpMatrixExponentialStateSpaceSystemDiscretizer(nStates, nInputs);
      solver = LinearSolverFactory.linear(nMeasurements);

      jointPositionVariance.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            CommonOps.setIdentity(R);
            double variance = jointPositionVariance.getDoubleValue();
            CommonOps.scale(variance * variance, R);
         }
      });

      processJerkVariance.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            // set up the process as constant accelerations for now
            CommonOps.fill(A, 0.0);
            A.set(0, 3, 1.0);
            A.set(1, 4, 1.0);
            A.set(2, 5, 1.0);
            A.set(3, 6, 1.0);
            A.set(4, 7, 1.0);
            A.set(5, 8, 1.0);

            // disable all inputs
            CommonOps.fill(B, 0.0);

            // set up the noise matrices
            double variance = processJerkVariance.getDoubleValue();
            CommonOps.setIdentity(R);
            CommonOps.fill(Q, 0.0);
            Q.set(6, 6, variance * variance);
            Q.set(7, 7, variance * variance);
            Q.set(8, 8, variance * variance);

            discretizer.discretize(A, B, Q, dt);
         }
      });

      // set the noise parameters
      processJerkVariance.set(100.0);
      jointPositionVariance.set(0.01);

      // set up the measurement matrix
      CommonOps.fill(H, 0.0);
      H.set(0, 0, 1.0);
      H.set(1, 1, 1.0);
      H.set(2, 2, 1.0);

      // initialize the error covariance
      CommonOps.setIdentity(PPosteriori);

   }

   @Override
   public void doControl()
   {
      updateValues();
      doKalmanUpdate();
      saveState();
   }

   private final DenseMatrix64F APA = new DenseMatrix64F(nStates, nStates);
   private final DenseMatrix64F HPH = new DenseMatrix64F(nMeasurements, nMeasurements);
   private final DenseMatrix64F HPHplusR = new DenseMatrix64F(nMeasurements, nMeasurements);
   private final DenseMatrix64F HPHplusRinverse = new DenseMatrix64F(nMeasurements, nMeasurements);
   private final DenseMatrix64F PH = new DenseMatrix64F(nStates, nMeasurements);
   private final DenseMatrix64F Hx = new DenseMatrix64F(nMeasurements, 1);
   private final DenseMatrix64F residual = new DenseMatrix64F(nMeasurements, 1);
   private final DenseMatrix64F Kresidual = new DenseMatrix64F(nStates, 1);
   private final DenseMatrix64F identity = new DenseMatrix64F(nStates, nStates);
   private final DenseMatrix64F KH = new DenseMatrix64F(nStates, nStates);
   private final DenseMatrix64F IminusKH = new DenseMatrix64F(nStates, nStates);

   private final void doKalmanUpdate()
   {
      // time update
      CommonOps.mult(A, xPosteriori, xPriori);
      CommonOps.multAdd(B, u, xPriori);

      multBothSides(PPosteriori, A, APA);
      CommonOps.add(APA, Q, PPriori);

      // measurement update
      multBothSides(PPriori, H, HPH);
      CommonOps.add(HPH, R, HPHplusR);
      if (!solver.setA(HPHplusR))
         throw new SingularMatrixException();
      solver.invert(HPHplusRinverse);
      CommonOps.multTransB(PPriori, H, PH);
      CommonOps.mult(PH, HPHplusRinverse, K);

      CommonOps.mult(H, xPriori, Hx);
      CommonOps.subtract(z, Hx, residual);
      CommonOps.mult(K, residual, Kresidual);
      CommonOps.add(xPriori, Kresidual, xPosteriori);

      CommonOps.setIdentity(identity);
      CommonOps.mult(K, H, KH);
      CommonOps.subtract(identity, KH, IminusKH);
      CommonOps.mult(IminusKH, PPriori, PPosteriori);
   }

   private final void updateValues()
   {
      robot.readFromSimulation();
      z.set(0, robot.getYaw());
      z.set(1, robot.getPitch1());
      z.set(2, robot.getPitch2());

      yoX.get(xPosteriori);
      yoP.get(PPosteriori);

      CommonOps.fill(u, 0.0);
   }

   private final void saveState()
   {
      yoX.set(xPosteriori);
      yoP.set(PPosteriori);
   }

   private final DenseMatrix64F tempForMultBothSides = new DenseMatrix64F(nStates, nStates);
   /** c = b * a * b' */
   private void multBothSides(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c)
   {
      tempForMultBothSides.reshape(a.getNumRows(), b.getNumRows());
      CommonOps.fill(tempForMultBothSides, 0.0);
      CommonOps.multAddTransB(a, b, tempForMultBothSides);
      CommonOps.mult(b, tempForMultBothSides, c);
   }
}
