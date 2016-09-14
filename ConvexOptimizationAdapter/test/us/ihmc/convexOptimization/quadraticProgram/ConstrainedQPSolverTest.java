package us.ihmc.convexOptimization.quadraticProgram;

import static org.junit.Assert.assertTrue;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class ConstrainedQPSolverTest
{

   @DeployableTestMethod(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testSolveContrainedQP() throws NoConvergenceException
   {
      YoVariableRegistry registry = new YoVariableRegistry("root");

      //      ConstrainedQPSolver jOptimizerSolver = new JOptimizerConstrainedQPSolver();
      ConstrainedQPSolver oasesSolver = new OASESConstrainedQPSolver(registry);
      ConstrainedQPSolver quadProgSolver = new QuadProgSolver();
      ConstrainedQPSolver compositeSolver = new CompositeActiveSetQPSolver(registry);
      ConstrainedQPSolver ojAlgoSolver = new OJAlgoConstrainedQPSolver();

      // Minimize 1/2 (x1^2 + x2^2) + x1 subject to x1 + x2 = 0, 2x1 + x2 <= 0
      int numberOfVariables = 2;
      int numberOfInequalityConstraints = 1;
      int numberOfEqualityConstraints = 1;

      DenseMatrix64F QCostFunction = new DenseMatrix64F(numberOfVariables, numberOfVariables, true, 1, 0, 0, 1);
      DenseMatrix64F fCostFunction = new DenseMatrix64F(numberOfVariables, 1, true, 1, 0);
      DenseMatrix64F AEqualityConstraints = new DenseMatrix64F(numberOfEqualityConstraints, numberOfVariables, true, 1, 1);
      DenseMatrix64F bEqualityConstraints = new DenseMatrix64F(numberOfEqualityConstraints, 1, true, 0);
      DenseMatrix64F AInequalityConstraints = new DenseMatrix64F(numberOfInequalityConstraints, numberOfVariables, true, 2, 1);
      DenseMatrix64F bInequalityConstraints = new DenseMatrix64F(numberOfInequalityConstraints, 1, true, 0);

      ConstrainedQPSolver[] optimizers = new ConstrainedQPSolver[] { oasesSolver, quadProgSolver, compositeSolver, ojAlgoSolver };

      for (int repeat = 0; repeat < 1000; repeat++)
      {
         for (int i = 0; i < optimizers.length; i++)
         {
            DenseMatrix64F x = new DenseMatrix64F(numberOfVariables, 1, true, -1, 1);
            optimizers[i].solve(QCostFunction, fCostFunction, AEqualityConstraints, bEqualityConstraints, AInequalityConstraints, bInequalityConstraints, x, false);
            Assert.assertArrayEquals(new double[] { -0.5, 0.5 }, x.getData(), 1e-10);
         }
      }

      // Minimize 1/2 (x1^2 + x2^2) subject to x1 + x2 = 0, 2x1 - x2 = 0
      numberOfVariables = 2;
      numberOfInequalityConstraints = 0;
      numberOfEqualityConstraints = 2;

      QCostFunction = new DenseMatrix64F(numberOfVariables, numberOfVariables, true, 1, 0, 0, 1);
      fCostFunction = new DenseMatrix64F(numberOfVariables, 1, true, 1, 0);
      AEqualityConstraints = new DenseMatrix64F(numberOfEqualityConstraints, numberOfVariables, true, 1, 1, 1, -1);
      bEqualityConstraints = new DenseMatrix64F(numberOfEqualityConstraints, 1, true, 6, 2);
      AInequalityConstraints = null;
      bInequalityConstraints = null;

      optimizers = new ConstrainedQPSolver[] { oasesSolver, quadProgSolver, compositeSolver, ojAlgoSolver };

      for (int i = 0; i < optimizers.length; i++)
      {
         DenseMatrix64F x = new DenseMatrix64F(numberOfVariables, 1, true, -1, 1);
         optimizers[i].solve(QCostFunction, fCostFunction, AEqualityConstraints, bEqualityConstraints, AInequalityConstraints, bInequalityConstraints, x, false);
         Assert.assertArrayEquals(new double[] { 4.0, 2.0 }, x.getData(), 1e-10);
      }

      // Minimize 1/2 (x1^2 + x2^2) subject to x1 + x2 = 0, 2x1 - x2 = 0
      numberOfVariables = 3;
      numberOfEqualityConstraints = 1;
      numberOfInequalityConstraints = 0;

      QCostFunction = new DenseMatrix64F(numberOfVariables, numberOfVariables, true, 1, 0, 0, 0, 1, 0, 0, 0, 1);
      fCostFunction = new DenseMatrix64F(numberOfVariables, 1, true, 1, 0, 2);
      AEqualityConstraints = new DenseMatrix64F(numberOfEqualityConstraints, numberOfVariables, true, 1, 1, 1);
      bEqualityConstraints = new DenseMatrix64F(numberOfEqualityConstraints, 1, true, 10);
      AInequalityConstraints = null;
      bInequalityConstraints = null;

      optimizers = new ConstrainedQPSolver[] { oasesSolver, quadProgSolver, compositeSolver, ojAlgoSolver };

      for (int i = 0; i < optimizers.length; i++)
      {
         DenseMatrix64F x = new DenseMatrix64F(numberOfVariables, 1, true, 0, 0, 0);
         optimizers[i].solve(QCostFunction, fCostFunction, AEqualityConstraints, bEqualityConstraints, AInequalityConstraints, bInequalityConstraints, x, false);
         Assert.assertArrayEquals(new double[] { 3.33333333333, 4.33333333333333, 2.3333333333333333 }, x.getData(), 1e-10);
      }

      numberOfInequalityConstraints = 1;

      AInequalityConstraints = new DenseMatrix64F(numberOfInequalityConstraints, numberOfVariables, true, 2, 1, 3);
      bInequalityConstraints = new DenseMatrix64F(numberOfInequalityConstraints, 1, true, 0);

      optimizers = new ConstrainedQPSolver[] { oasesSolver, quadProgSolver, compositeSolver, ojAlgoSolver };

      for (int i = 0; i < optimizers.length; i++)
      {
         DenseMatrix64F x = new DenseMatrix64F(numberOfVariables, 1, true, 0, 0, 0);
         optimizers[i].solve(QCostFunction, fCostFunction, AEqualityConstraints, bEqualityConstraints, AInequalityConstraints, bInequalityConstraints, x, false);
         Assert.assertArrayEquals(new double[] { 3.33333333333, 13.33333333333333, -6.666666666666666 }, x.getData(), 1e-10);
      }

      numberOfInequalityConstraints = 1;

      AInequalityConstraints = new DenseMatrix64F(numberOfInequalityConstraints, numberOfVariables, true, 4, 5, 7);
      bInequalityConstraints = new DenseMatrix64F(numberOfInequalityConstraints, 1, true, 0);

      optimizers = new ConstrainedQPSolver[] { oasesSolver, quadProgSolver, compositeSolver };
//      optimizers = new ConstrainedQPSolver[] { ojAlgoSolver};

      double[] expectedSolution = new double[] { 18.0, 8.0, -16.0 };

      for (int i = 0; i < optimizers.length; i++)
      {
         DenseMatrix64F x = new DenseMatrix64F(numberOfVariables, 1, true, 0, 0, 0);
         optimizers[i].solve(QCostFunction, fCostFunction, AEqualityConstraints, bEqualityConstraints, AInequalityConstraints, bInequalityConstraints, x, false);
         Assert.assertArrayEquals(expectedSolution, x.getData(), 1e-10);
      }

      numberOfInequalityConstraints = 2;

      AInequalityConstraints = new DenseMatrix64F(numberOfInequalityConstraints, numberOfVariables, true, 2, 1, 3, 4, 5, 7);
      bInequalityConstraints = new DenseMatrix64F(numberOfInequalityConstraints, 1, true, 0, 6);

      expectedSolution = new double[] { 16.28571428571428, 7.571428571428572, -13.857142857142854 }; // oasesSolver, quadProgSolver, and compositeSolver

      for (int i = 0; i < optimizers.length; i++)
      {
         DenseMatrix64F x = new DenseMatrix64F(numberOfVariables, 1, true, 0, 0, 0);
         optimizers[i].solve(QCostFunction, fCostFunction, AEqualityConstraints, bEqualityConstraints, AInequalityConstraints, bInequalityConstraints, x, false);
         Assert.assertArrayEquals(expectedSolution, x.getData(), 1e-10);
      }
   }

   @DeployableTestMethod(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testSomeTrickyOnes() throws NoConvergenceException
   {
      YoVariableRegistry registry = new YoVariableRegistry("root");

      //TODO: Need more test cases. Can't trust these QP solvers without them...
      ConstrainedQPSolver oasesSolver = new OASESConstrainedQPSolver(registry);
      ConstrainedQPSolver quadProgSolver = new QuadProgSolver();
      ConstrainedQPSolver compositeSolver = new CompositeActiveSetQPSolver(registry);
      ConstrainedQPSolver ojAlgoSolver = new OJAlgoConstrainedQPSolver();

      ConstrainedQPSolver[] optimizers = new ConstrainedQPSolver[] { oasesSolver, quadProgSolver, compositeSolver};
//      ConstrainedQPSolver[] optimizers = new ConstrainedQPSolver[] {ojAlgoSolver};

      int numberOfInequalityConstraints = 1;
      int numberOfEqualityConstraints = 2;
      int numberOfVariables = 3;

      DenseMatrix64F QCostFunction = new DenseMatrix64F(numberOfVariables, numberOfVariables, true, 1, 0, 1, 0, 1, 2, 1, 2, 7);
      DenseMatrix64F fCostFunction = new DenseMatrix64F(numberOfVariables, 1, true, 1, 0, 9);
      DenseMatrix64F AEqualityConstraints = new DenseMatrix64F(numberOfEqualityConstraints, numberOfVariables, true, 1, 1, 1, 2, 3, 4);
      DenseMatrix64F bEqualityConstraints = new DenseMatrix64F(numberOfEqualityConstraints, 1, true, 0, 7);
      DenseMatrix64F AInequalityConstraints = new DenseMatrix64F(numberOfInequalityConstraints, numberOfVariables, true, 2, 1, 3);
      DenseMatrix64F bInequalityConstraints = new DenseMatrix64F(numberOfInequalityConstraints, 1, true, 0);

      for (int repeat = 0; repeat < 1000; repeat++)
      {
         for (int i = 0; i < optimizers.length; i++)
         {
            DenseMatrix64F x = new DenseMatrix64F(numberOfVariables, 1, true, -1, 1, 3);
            optimizers[i].solve(QCostFunction, fCostFunction, AEqualityConstraints, bEqualityConstraints, AInequalityConstraints, bInequalityConstraints, x, false);
            double[] expectedAnswer = new double[] { -6.333333333333, 5.666666666666666, 0.666666666666666666 }; //quadProgSolver and oasesSolver and compositeSolver
//            double[] expectedAnswer = new double[] { 0.0, 10.5, -3.5}; // ojAlgoSolver. Not consistent with equality constraints!?
            Assert.assertArrayEquals("repeat = " + repeat + ", optimizer = " + i, expectedAnswer, x.getData(), 1e-10);

            DenseMatrix64F bEqualityVerify = new DenseMatrix64F(numberOfEqualityConstraints, 1);
            CommonOps.mult(AEqualityConstraints, x, bEqualityVerify);

            // Verify Ax=b Equality constraints hold:
            JUnitTools.assertMatrixEquals(bEqualityConstraints, bEqualityVerify, 1e-7);

            // Verify Ax<b Inequality constraints hold:
            DenseMatrix64F bInequalityVerify = new DenseMatrix64F(numberOfInequalityConstraints, 1);
            CommonOps.mult(AInequalityConstraints, x, bInequalityVerify);

            for (int j = 0; j < bInequalityVerify.getNumRows(); j++)
            {
               assertTrue(bInequalityVerify.get(j, 0) < bInequalityConstraints.get(j, 0));
            }
         }
      }
   }

}
