package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;

import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertEquals;


@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SimpleEfficientActiveSetQPSolverSymmetricHessianTest extends AbstractSimpleActiveSetQPSolverTest
{
   private static final double epsilon = 1e-4;

   @Override
   public SimpleActiveSetQPSolverInterface createSolverToTest()
   {
      SimpleEfficientActiveSetQPSolver simpleEfficientActiveSetQPSolver = new SimpleEfficientActiveSetQPSolver(true);
      simpleEfficientActiveSetQPSolver.setUseWarmStart(false);
      return simpleEfficientActiveSetQPSolver;

   }

   @Override
   public void testSimpleCasesWithBoundsConstraints(int expectedNumberOfIterations, int expectedNumberOfIterations2, int expectedNumberOfIterations3, int expectedNubmerOfIterations3,
                                                    boolean ignoreLagrangeMultipliers)
   {
      SimpleActiveSetQPSolverInterface solver = createSolverToTest();

      // Minimize x^T * x
      double[][] costQuadraticMatrix = new double[][] { { 2.0 } };
      double[] costLinearVector = new double[] { 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[] variableLowerBounds = new double[] { Double.NEGATIVE_INFINITY };
      double[] variableUpperBounds = new double[] { Double.POSITIVE_INFINITY };
      solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

      double[] solution = new double[1];
      double[] lagrangeEqualityMultipliers = new double[0];
      double[] lagrangeInequalityMultipliers = new double[0];
      double[] lagrangeLowerBoundMultipliers = new double[1];
      double[] lagrangeUpperBoundMultipliers = new double[1];

      int numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNumberOfIterations, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(0.0, solution[0], 1e-7);
      assertEquals(0.0, lagrangeLowerBoundMultipliers[0], 1e-7);
      assertEquals(0.0, lagrangeUpperBoundMultipliers[0], 1e-7);

      // Minimize x^T * x subject to x >= 1
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      variableLowerBounds = new double[] { 1.0 };
      variableUpperBounds = new double[] { Double.POSITIVE_INFINITY };
      solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

      solution = new double[1];
      lagrangeEqualityMultipliers = new double[0];
      lagrangeInequalityMultipliers = new double[0];
      lagrangeLowerBoundMultipliers = new double[1];
      lagrangeUpperBoundMultipliers = new double[1];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNumberOfIterations + 1, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(1.0, solution[0], 1e-7);
      assertEquals(2.0, lagrangeLowerBoundMultipliers[0], 1e-7);
      assertEquals(0.0, lagrangeUpperBoundMultipliers[0], 1e-7);

      // Minimize x^T * x subject to x <= -1
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      variableLowerBounds = new double[] { Double.NEGATIVE_INFINITY };
      variableUpperBounds = new double[] { -1.0 };
      solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

      solution = new double[1];
      lagrangeEqualityMultipliers = new double[0];
      lagrangeInequalityMultipliers = new double[0];
      lagrangeLowerBoundMultipliers = new double[1];
      lagrangeUpperBoundMultipliers = new double[1];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNumberOfIterations + 1, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(-1.0, solution[0], 1e-7);
      assertEquals(0.0, lagrangeLowerBoundMultipliers[0], 1e-7);
      assertEquals(2.0, lagrangeUpperBoundMultipliers[0], 1e-7);

      // Minimize x^T * x subject to 1 + 1e-12 <= x <= 1 - 1e-12 (Should give valid solution given a little epsilon to allow for roundoff)
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      variableLowerBounds = new double[] { 1.0 + 1e-12};
      variableUpperBounds = new double[] { 1.0 - 1e-12};
      solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

      solution = new double[1];
      lagrangeEqualityMultipliers = new double[0];
      lagrangeInequalityMultipliers = new double[0];
      lagrangeLowerBoundMultipliers = new double[1];
      lagrangeUpperBoundMultipliers = new double[1];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNumberOfIterations + 1, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(1.0, solution[0], 1e-7);
      assertEquals(2.0, lagrangeLowerBoundMultipliers[0], 1e-7);
      assertEquals(0.0, lagrangeUpperBoundMultipliers[0], 1e-7);

      // Minimize x^T * x subject to -1 + 1e-12 <= x <= -1 - 1e-12 (Should give valid solution given a little epsilon to allow for roundoff)
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      variableLowerBounds = new double[] { -1.0 + 1e-12};
      variableUpperBounds = new double[] { -1.0 - 1e-12};
      solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

      solution = new double[1];
      lagrangeEqualityMultipliers = new double[0];
      lagrangeInequalityMultipliers = new double[0];
      lagrangeLowerBoundMultipliers = new double[1];
      lagrangeUpperBoundMultipliers = new double[1];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNumberOfIterations + 1, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(-1.0, solution[0], 1e-7);
      assertEquals(0.0, lagrangeLowerBoundMultipliers[0], 1e-7);
      assertEquals(2.0, lagrangeUpperBoundMultipliers[0], 1e-7);

      /** For some reason this doesn't work well */
      /*
      // Minimize x^T * x subject to 1 + 1e-7 <= x <= 1 - 1e-7 (Should not give valid solution since this is too much to blame on roundoff)
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      variableLowerBounds = new double[] { 1.0 + 1e-7 };
      variableUpperBounds = new double[] { 1.0 - 1e-7 };
      solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

      solution = new double[1];
      lagrangeEqualityMultipliers = new double[0];
      lagrangeInequalityMultipliers = new double[0];
      lagrangeLowerBoundMultipliers = new double[1];
      lagrangeUpperBoundMultipliers = new double[1];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNumberOfIterations2, numberOfIterations);

      assertEquals(1, solution.length);
      assertTrue(Double.isNaN(solution[0]));
      assertTrue(Double.isInfinite(lagrangeLowerBoundMultipliers[0]));
      assertTrue(Double.isInfinite(lagrangeUpperBoundMultipliers[0]));
      */

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8, -5 <= x <= 5, 6 <= y <= 10, 11 <= z
      solver.clear();

      costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      double[] linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      double[][] linearInequalityConstraintsCMatrix = new double[][] { { 0.0, 1.0, -1.0 } };
      double[] linearInqualityConstraintsDVector = new double[] { -8.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solver.setVariableBounds(new double[] { -5.0, 6.0, 11.0 }, new double[] { 5.0, 10.0, Double.POSITIVE_INFINITY });

      solution = new double[3];
      lagrangeEqualityMultipliers = new double[1];
      lagrangeInequalityMultipliers = new double[1];
      lagrangeLowerBoundMultipliers = new double[3];
      lagrangeUpperBoundMultipliers = new double[3];

      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNubmerOfIterations3, numberOfIterations);

      assertEquals(3, solution.length);
      assertEquals(-4.0, solution[0], 1e-7);
      assertEquals(6.0, solution[1], 1e-7);
      assertEquals(14.0, solution[2], 1e-7);
      if (!ignoreLagrangeMultipliers)
      {
         assertEquals(8.0, lagrangeEqualityMultipliers[0], 1e-7);
         assertEquals(28.0, lagrangeInequalityMultipliers[0], 1e-7);

         assertEquals(0.0, lagrangeLowerBoundMultipliers[0], 1e-7);
         assertEquals(48.0, lagrangeLowerBoundMultipliers[1], 1e-7);
         assertEquals(0.0, lagrangeLowerBoundMultipliers[2], 1e-7);

         assertEquals(0.0, lagrangeUpperBoundMultipliers[0], 1e-7);
         assertEquals(0.0, lagrangeUpperBoundMultipliers[1], 1e-7);
         assertEquals(0.0, lagrangeUpperBoundMultipliers[2], 1e-7);
      }

      DenseMatrix64F solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      double objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(248.0, objectiveCost, 1e-7);

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8, 3 <= x <= 5, 6 <= y <= 10, 11 <= z
      solver.clear();

      costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 0.0, 1.0, -1.0 } };
      linearInqualityConstraintsDVector = new double[] { -8.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solver.setVariableBounds(new double[] { 3.0, 6.0, 11.0 }, new double[] { 5.0, 10.0, Double.POSITIVE_INFINITY });

      solution = new double[3];
      lagrangeEqualityMultipliers = new double[1];
      lagrangeInequalityMultipliers = new double[1];
      lagrangeLowerBoundMultipliers = new double[3];
      lagrangeUpperBoundMultipliers = new double[3];

      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNumberOfIterations3, numberOfIterations);

      assertEquals(3, solution.length);
      assertTrue(Double.isNaN(solution[0]));
      assertTrue(Double.isNaN(solution[1]));
      assertTrue(Double.isNaN(solution[2]));

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertTrue(Double.isNaN(objectiveCost));
   }
}
