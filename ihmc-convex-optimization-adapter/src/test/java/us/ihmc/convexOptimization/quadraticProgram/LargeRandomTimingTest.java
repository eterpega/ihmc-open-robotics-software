package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.random.RandomGeometry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertEquals;

public class LargeRandomTimingTest
{
   private static final boolean VERBOSE = true;

   private List<SimpleActiveSetQPSolverInterface> createSolversToTest()
   {
      ArrayList<SimpleActiveSetQPSolverInterface> tests = new ArrayList<>();

      JavaQuadProgSolver solver = new JavaQuadProgSolver();
      solver.setUseWarmStart(false);
      tests.add(solver);

      SimpleEfficientActiveSetQPSolver symmetricSolver = new SimpleEfficientActiveSetQPSolver(true);
      symmetricSolver.setUseWarmStart(false);
      tests.add(symmetricSolver);

      SimpleEfficientActiveSetQPSolver regularSolver = new SimpleEfficientActiveSetQPSolver();
      regularSolver.setUseWarmStart(false);
      tests.add(regularSolver);



      return tests;
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 3000000)
   public void testLargeRandomProblemWithInequalityAndBoundsConstraints()
   {
      int numberOfTests = 1200;
      List<SimpleActiveSetQPSolverInterface> solvers = createSolversToTest();

      for (SimpleActiveSetQPSolverInterface solver : solvers)
      {
         Random random = new Random(1776L);

         long startTimeMillis = System.currentTimeMillis();
         int maxNumberOfIterations = 0;

         int numberOfVariables = 80;
         int numberOfEqualityConstraints = 10;
         int numberOfInequalityConstraints = 36;

         DenseMatrix64F solution = new DenseMatrix64F(0, 0);
         DenseMatrix64F lagrangeEqualityMultipliers = new DenseMatrix64F(0, 0);
         DenseMatrix64F lagrangeInequalityMultipliers = new DenseMatrix64F(0, 0);
         DenseMatrix64F lagrangeLowerBoundMultipliers = new DenseMatrix64F(0, 0);
         DenseMatrix64F lagrangeUpperBoundMultipliers = new DenseMatrix64F(0, 0);
         double[] solutionWithSmallPerturbation = new double[numberOfVariables];

         DenseMatrix64F augmentedLinearEqualityConstraintsAMatrix = new DenseMatrix64F(0, 0);
         DenseMatrix64F augmentedLinearEqualityConstraintsBVector = new DenseMatrix64F(0, 0);

         int numberOfNaNSolutions = 0;
         for (int testNumber = 0; testNumber < numberOfTests; testNumber++)

         {
            //         System.out.println("testNumber = " + testNumber);
            solver.clear();

            DenseMatrix64F costQuadraticMatrix = RandomGeometry.nextDenseMatrix64F(random, numberOfVariables, numberOfVariables);
            DenseMatrix64F identity = CommonOps.identity(numberOfVariables, numberOfVariables); // Add n*I to make sure it is positive definite...
            CommonOps.scale(numberOfVariables, identity);
            CommonOps.addEquals(costQuadraticMatrix, identity);

            DenseMatrix64F costLinearVector = RandomGeometry.nextDenseMatrix64F(random, numberOfVariables, 1);
            double quadraticCostScalar = RandomNumbers.nextDouble(random, 30.0);

            solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

            DenseMatrix64F linearEqualityConstraintsAMatrix = RandomGeometry.nextDenseMatrix64F(random, numberOfEqualityConstraints, numberOfVariables);
            DenseMatrix64F linearEqualityConstraintsBVector = RandomGeometry.nextDenseMatrix64F(random, numberOfEqualityConstraints, 1);
            solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

            DenseMatrix64F linearInequalityConstraintsCMatrix = RandomGeometry.nextDenseMatrix64F(random, numberOfInequalityConstraints, numberOfVariables);
            DenseMatrix64F linearInequalityConstraintsDVector = RandomGeometry.nextDenseMatrix64F(random, numberOfInequalityConstraints, 1);
            solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector);

            DenseMatrix64F variableLowerBounds = RandomGeometry.nextDenseMatrix64F(random, numberOfVariables, 1, -5.0, -0.01);
            DenseMatrix64F variableUpperBounds = RandomGeometry.nextDenseMatrix64F(random, numberOfVariables, 1, 0.01, 5.0);
            solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

            int numberOfIterations = solver
                  .solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
            if (numberOfIterations > maxNumberOfIterations)
               maxNumberOfIterations = numberOfIterations;

            //         System.out.println(solution);
            //         System.out.println("numberOfIterations = " + numberOfIterations);

            assertEquals(numberOfVariables, solution.getNumRows());
            assertEquals(numberOfEqualityConstraints, lagrangeEqualityMultipliers.getNumRows());
            assertEquals(numberOfInequalityConstraints, lagrangeInequalityMultipliers.getNumRows());
            assertEquals(variableLowerBounds.getNumRows(), lagrangeLowerBoundMultipliers.getNumRows());
            assertEquals(variableUpperBounds.getNumRows(), lagrangeUpperBoundMultipliers.getNumRows());

            if (Double.isNaN(solution.get(0)))
            {
               numberOfNaNSolutions++;
               continue;
            }

            double objectiveCost = solver.getObjectiveCost(solution);

            // Verify constraints hold:
            verifyEqualityConstraintsHold(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solution);
            verifyInequalityConstraintsHold(numberOfInequalityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solution);
            verifyVariableBoundsHold(testNumber, variableLowerBounds, variableUpperBounds, solution);

            // Verify objective is minimized by comparing to small perturbation:
            for (int i = 0; i < numberOfVariables; i++)
            {
               solutionWithSmallPerturbation[i] = solution.get(i, 0) + RandomNumbers.nextDouble(random, 5e-3);
            }

            solution.zero();
            solution.setData(solutionWithSmallPerturbation);

            verifyEqualityConstraintsDoNotHold(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solution);

            // Equality constraints usually do not hold. Sometimes they do, so if you run with lots of numberOfTests, comment out the following:
            verifyInequalityConstraintsDoNotHold(numberOfInequalityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solution);

            int activeInequalitiesSize = 0;
            for (int i = 0; i < numberOfInequalityConstraints; i++)
            {
               double lagrangeMultiplier = lagrangeInequalityMultipliers.get(i, 0);

               if (lagrangeMultiplier < 0.0)
               {
                  throw new RuntimeException("Received a negative lagrange multiplier.");
               }
               if (lagrangeMultiplier > 0.0)
               {
                  activeInequalitiesSize++;
               }
            }

            int activeLowerBoundsSize = 0;
            for (int i = 0; i < variableLowerBounds.getNumRows(); i++)
            {
               double lagrangeMultiplier = lagrangeLowerBoundMultipliers.get(i, 0);

               if (lagrangeMultiplier < 0.0)
               {
                  throw new RuntimeException();
               }
               if (lagrangeMultiplier > 0.0)
               {
                  activeLowerBoundsSize++;
               }
            }

            int activeUpperBoundsSize = 0;
            for (int i = 0; i < variableUpperBounds.getNumRows(); i++)
            {
               double lagrangeMultiplier = lagrangeUpperBoundMultipliers.get(i, 0);

               if (lagrangeMultiplier < 0.0)
               {
                  throw new RuntimeException();
               }
               if (lagrangeMultiplier > 0.0)
               {
                  activeUpperBoundsSize++;
               }
            }

            augmentedLinearEqualityConstraintsAMatrix
                  .reshape(numberOfEqualityConstraints + activeInequalitiesSize + activeLowerBoundsSize + activeUpperBoundsSize, numberOfVariables);
            augmentedLinearEqualityConstraintsBVector
                  .reshape(numberOfEqualityConstraints + activeInequalitiesSize + activeLowerBoundsSize + activeUpperBoundsSize, 1);
            augmentedLinearEqualityConstraintsAMatrix.zero();
            augmentedLinearEqualityConstraintsBVector.zero();

            CommonOps
                  .extract(linearEqualityConstraintsAMatrix, 0, numberOfEqualityConstraints, 0, numberOfVariables, augmentedLinearEqualityConstraintsAMatrix, 0,
                           0);
            CommonOps.extract(linearEqualityConstraintsBVector, 0, numberOfEqualityConstraints, 0, 1, augmentedLinearEqualityConstraintsBVector, 0, 0);

            int index = 0;
            for (int i = 0; i < numberOfInequalityConstraints; i++)
            {
               double lagrangeMultiplier = lagrangeInequalityMultipliers.get(i, 0);

               if (lagrangeMultiplier < 0.0)
               {
                  throw new RuntimeException();
               }
               if (lagrangeMultiplier > 0.0)
               {
                  CommonOps.extract(linearInequalityConstraintsCMatrix, i, i + 1, 0, numberOfVariables, augmentedLinearEqualityConstraintsAMatrix,
                                    numberOfEqualityConstraints + index, 0);
                  CommonOps.extract(linearInequalityConstraintsDVector, i, i + 1, 0, 1, augmentedLinearEqualityConstraintsBVector, numberOfEqualityConstraints + index, 0);
                  index++;
               }
            }

            for (int i = 0; i < variableLowerBounds.getNumRows(); i++)
            {
               double lagrangeMultiplier = lagrangeLowerBoundMultipliers.get(i, 0);

               if (lagrangeMultiplier > 0.0)
               {
                  augmentedLinearEqualityConstraintsAMatrix.set(numberOfEqualityConstraints + index, i, -1.0);
                  augmentedLinearEqualityConstraintsBVector.set(numberOfEqualityConstraints + index, -variableLowerBounds.get(i));
                  index++;
               }
            }

            for (int i = 0; i < variableUpperBounds.getNumRows(); i++)
            {
               double lagrangeMultiplier = lagrangeUpperBoundMultipliers.get(i, 0);

               if (lagrangeMultiplier > 0.0)
               {
                  augmentedLinearEqualityConstraintsAMatrix.set(numberOfEqualityConstraints + index, i, 1.0);
                  augmentedLinearEqualityConstraintsBVector.set(numberOfEqualityConstraints + index, variableUpperBounds.get(i));
                  index++;
               }
            }

            assertTrue(index == activeInequalitiesSize + activeLowerBoundsSize + activeUpperBoundsSize);

            DenseMatrix64F solutionMatrixProjectedOntoEqualityConstraints = projectOntoEqualityConstraints(solution, augmentedLinearEqualityConstraintsAMatrix,
                                                                                                           augmentedLinearEqualityConstraintsBVector);
            verifyEqualityConstraintsHold(numberOfEqualityConstraints + activeInequalitiesSize + activeLowerBoundsSize + activeUpperBoundsSize,
                                          augmentedLinearEqualityConstraintsAMatrix, augmentedLinearEqualityConstraintsBVector,
                                          solutionMatrixProjectedOntoEqualityConstraints);

            double maxSignedError = getMaxInequalityConstraintError(numberOfInequalityConstraints, linearInequalityConstraintsCMatrix,
                                                                    linearInequalityConstraintsDVector, solutionMatrixProjectedOntoEqualityConstraints);

            double objectiveCostWithSmallPerturbation = solver.getObjectiveCost(solutionMatrixProjectedOntoEqualityConstraints);

            if (maxSignedError < 1.0e-7) // Java quad prog does not necessarily include the correct form of equality constraints, so this must be considered.
            {
               assertTrue("objectiveCostWithSmallPerturbation = " + objectiveCostWithSmallPerturbation + ", objectiveCost = " + objectiveCost,
                          objectiveCostWithSmallPerturbation > objectiveCost);
            }
         }

         assertTrue(numberOfNaNSolutions < 0.05 * numberOfTests);

         long endTimeMillis = System.currentTimeMillis();

         double timePerTest = ((double) (endTimeMillis - startTimeMillis)) * 0.001 / ((double) numberOfTests);
         // considered warmup
      }
      for (SimpleActiveSetQPSolverInterface solver : solvers)
      {
         Random random = new Random(1776L);

         long startTimeMillis = System.currentTimeMillis();
         int maxNumberOfIterations = 0;

         int numberOfVariables = 80;
         int numberOfEqualityConstraints = 10;
         int numberOfInequalityConstraints = 36;

         DenseMatrix64F solution = new DenseMatrix64F(0, 0);
         DenseMatrix64F lagrangeEqualityMultipliers = new DenseMatrix64F(0, 0);
         DenseMatrix64F lagrangeInequalityMultipliers = new DenseMatrix64F(0, 0);
         DenseMatrix64F lagrangeLowerBoundMultipliers = new DenseMatrix64F(0, 0);
         DenseMatrix64F lagrangeUpperBoundMultipliers = new DenseMatrix64F(0, 0);
         double[] solutionWithSmallPerturbation = new double[numberOfVariables];

         DenseMatrix64F augmentedLinearEqualityConstraintsAMatrix = new DenseMatrix64F(0, 0);
         DenseMatrix64F augmentedLinearEqualityConstraintsBVector = new DenseMatrix64F(0, 0);

         int numberOfNaNSolutions = 0;
         for (int testNumber = 0; testNumber < numberOfTests; testNumber++)

         {
            //         System.out.println("testNumber = " + testNumber);
            solver.clear();

            DenseMatrix64F costQuadraticMatrix = RandomGeometry.nextDenseMatrix64F(random, numberOfVariables, numberOfVariables);
            DenseMatrix64F identity = CommonOps.identity(numberOfVariables, numberOfVariables); // Add n*I to make sure it is positive definite...
            CommonOps.scale(numberOfVariables, identity);
            CommonOps.addEquals(costQuadraticMatrix, identity);

            DenseMatrix64F costLinearVector = RandomGeometry.nextDenseMatrix64F(random, numberOfVariables, 1);
            double quadraticCostScalar = RandomNumbers.nextDouble(random, 30.0);

            solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

            DenseMatrix64F linearEqualityConstraintsAMatrix = RandomGeometry.nextDenseMatrix64F(random, numberOfEqualityConstraints, numberOfVariables);
            DenseMatrix64F linearEqualityConstraintsBVector = RandomGeometry.nextDenseMatrix64F(random, numberOfEqualityConstraints, 1);
            solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

            DenseMatrix64F linearInequalityConstraintsCMatrix = RandomGeometry.nextDenseMatrix64F(random, numberOfInequalityConstraints, numberOfVariables);
            DenseMatrix64F linearInequalityConstraintsDVector = RandomGeometry.nextDenseMatrix64F(random, numberOfInequalityConstraints, 1);
            solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector);

            DenseMatrix64F variableLowerBounds = RandomGeometry.nextDenseMatrix64F(random, numberOfVariables, 1, -5.0, -0.01);
            DenseMatrix64F variableUpperBounds = RandomGeometry.nextDenseMatrix64F(random, numberOfVariables, 1, 0.01, 5.0);
            solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

            int numberOfIterations = solver
                  .solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
            if (numberOfIterations > maxNumberOfIterations)
               maxNumberOfIterations = numberOfIterations;

            //         System.out.println(solution);
            //         System.out.println("numberOfIterations = " + numberOfIterations);

            assertEquals(numberOfVariables, solution.getNumRows());
            assertEquals(numberOfEqualityConstraints, lagrangeEqualityMultipliers.getNumRows());
            assertEquals(numberOfInequalityConstraints, lagrangeInequalityMultipliers.getNumRows());
            assertEquals(variableLowerBounds.getNumRows(), lagrangeLowerBoundMultipliers.getNumRows());
            assertEquals(variableUpperBounds.getNumRows(), lagrangeUpperBoundMultipliers.getNumRows());

            if (Double.isNaN(solution.get(0)))
            {
               numberOfNaNSolutions++;
               continue;
            }

            double objectiveCost = solver.getObjectiveCost(solution);

            // Verify constraints hold:
            verifyEqualityConstraintsHold(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solution);
            verifyInequalityConstraintsHold(numberOfInequalityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solution);
            verifyVariableBoundsHold(testNumber, variableLowerBounds, variableUpperBounds, solution);

            // Verify objective is minimized by comparing to small perturbation:
            for (int i = 0; i < numberOfVariables; i++)
            {
               solutionWithSmallPerturbation[i] = solution.get(i, 0) + RandomNumbers.nextDouble(random, 5e-3);
            }

            solution.zero();
            solution.setData(solutionWithSmallPerturbation);

            verifyEqualityConstraintsDoNotHold(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solution);

            // Equality constraints usually do not hold. Sometimes they do, so if you run with lots of numberOfTests, comment out the following:
            verifyInequalityConstraintsDoNotHold(numberOfInequalityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solution);

            int activeInequalitiesSize = 0;
            for (int i = 0; i < numberOfInequalityConstraints; i++)
            {
               double lagrangeMultiplier = lagrangeInequalityMultipliers.get(i, 0);

               if (lagrangeMultiplier < 0.0)
               {
                  throw new RuntimeException("Received a negative lagrange multiplier.");
               }
               if (lagrangeMultiplier > 0.0)
               {
                  activeInequalitiesSize++;
               }
            }

            int activeLowerBoundsSize = 0;
            for (int i = 0; i < variableLowerBounds.getNumRows(); i++)
            {
               double lagrangeMultiplier = lagrangeLowerBoundMultipliers.get(i, 0);

               if (lagrangeMultiplier < 0.0)
               {
                  throw new RuntimeException();
               }
               if (lagrangeMultiplier > 0.0)
               {
                  activeLowerBoundsSize++;
               }
            }

            int activeUpperBoundsSize = 0;
            for (int i = 0; i < variableUpperBounds.getNumRows(); i++)
            {
               double lagrangeMultiplier = lagrangeUpperBoundMultipliers.get(i, 0);

               if (lagrangeMultiplier < 0.0)
               {
                  throw new RuntimeException();
               }
               if (lagrangeMultiplier > 0.0)
               {
                  activeUpperBoundsSize++;
               }
            }

            augmentedLinearEqualityConstraintsAMatrix
                  .reshape(numberOfEqualityConstraints + activeInequalitiesSize + activeLowerBoundsSize + activeUpperBoundsSize, numberOfVariables);
            augmentedLinearEqualityConstraintsBVector
                  .reshape(numberOfEqualityConstraints + activeInequalitiesSize + activeLowerBoundsSize + activeUpperBoundsSize, 1);
            augmentedLinearEqualityConstraintsAMatrix.zero();
            augmentedLinearEqualityConstraintsBVector.zero();

            CommonOps
                  .extract(linearEqualityConstraintsAMatrix, 0, numberOfEqualityConstraints, 0, numberOfVariables, augmentedLinearEqualityConstraintsAMatrix, 0,
                           0);
            CommonOps.extract(linearEqualityConstraintsBVector, 0, numberOfEqualityConstraints, 0, 1, augmentedLinearEqualityConstraintsBVector, 0, 0);

            int index = 0;
            for (int i = 0; i < numberOfInequalityConstraints; i++)
            {
               double lagrangeMultiplier = lagrangeInequalityMultipliers.get(i, 0);

               if (lagrangeMultiplier < 0.0)
               {
                  throw new RuntimeException();
               }
               if (lagrangeMultiplier > 0.0)
               {
                  CommonOps.extract(linearInequalityConstraintsCMatrix, i, i + 1, 0, numberOfVariables, augmentedLinearEqualityConstraintsAMatrix,
                                    numberOfEqualityConstraints + index, 0);
                  CommonOps.extract(linearInequalityConstraintsDVector, i, i + 1, 0, 1, augmentedLinearEqualityConstraintsBVector, numberOfEqualityConstraints + index, 0);
                  index++;
               }
            }

            for (int i = 0; i < variableLowerBounds.getNumRows(); i++)
            {
               double lagrangeMultiplier = lagrangeLowerBoundMultipliers.get(i, 0);

               if (lagrangeMultiplier > 0.0)
               {
                  augmentedLinearEqualityConstraintsAMatrix.set(numberOfEqualityConstraints + index, i, -1.0);
                  augmentedLinearEqualityConstraintsBVector.set(numberOfEqualityConstraints + index, -variableLowerBounds.get(i));
                  index++;
               }
            }

            for (int i = 0; i < variableUpperBounds.getNumRows(); i++)
            {
               double lagrangeMultiplier = lagrangeUpperBoundMultipliers.get(i, 0);

               if (lagrangeMultiplier > 0.0)
               {
                  augmentedLinearEqualityConstraintsAMatrix.set(numberOfEqualityConstraints + index, i, 1.0);
                  augmentedLinearEqualityConstraintsBVector.set(numberOfEqualityConstraints + index, variableUpperBounds.get(i));
                  index++;
               }
            }

            assertTrue(index == activeInequalitiesSize + activeLowerBoundsSize + activeUpperBoundsSize);

            DenseMatrix64F solutionMatrixProjectedOntoEqualityConstraints = projectOntoEqualityConstraints(solution, augmentedLinearEqualityConstraintsAMatrix,
                                                                                                           augmentedLinearEqualityConstraintsBVector);
            verifyEqualityConstraintsHold(numberOfEqualityConstraints + activeInequalitiesSize + activeLowerBoundsSize + activeUpperBoundsSize,
                                          augmentedLinearEqualityConstraintsAMatrix, augmentedLinearEqualityConstraintsBVector,
                                          solutionMatrixProjectedOntoEqualityConstraints);

            double maxSignedError = getMaxInequalityConstraintError(numberOfInequalityConstraints, linearInequalityConstraintsCMatrix,
                                                                    linearInequalityConstraintsDVector, solutionMatrixProjectedOntoEqualityConstraints);

            double objectiveCostWithSmallPerturbation = solver.getObjectiveCost(solutionMatrixProjectedOntoEqualityConstraints);

            if (maxSignedError < 1.0e-7) // Java quad prog does not necessarily include the correct form of equality constraints, so this must be considered.
            {
               assertTrue("objectiveCostWithSmallPerturbation = " + objectiveCostWithSmallPerturbation + ", objectiveCost = " + objectiveCost,
                          objectiveCostWithSmallPerturbation > objectiveCost);
            }
         }

         assertTrue(numberOfNaNSolutions < 0.05 * numberOfTests);

         long endTimeMillis = System.currentTimeMillis();

         double timePerTest = ((double) (endTimeMillis - startTimeMillis)) * 0.001 / ((double) numberOfTests);
         if (VERBOSE)

         {
            System.out.println("Time per test is " + timePerTest);
            System.out.println("maxNumberOfIterations is " + maxNumberOfIterations);
            System.out.println("numberOfNaNSolutions = " + numberOfNaNSolutions);
            System.out.println("numberOfTests = " + numberOfTests);

         }
      }
   }

   private void verifyEqualityConstraintsHold(int numberOfEqualityConstraints, DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector, DenseMatrix64F solutionMatrix)
   {
      double maxAbsoluteError = getMaxEqualityConstraintError(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solutionMatrix);
      assertEquals(0.0, maxAbsoluteError, 1e-5);
   }

   private void verifyInequalityConstraintsHold(int numberOfEqualityConstraints, DenseMatrix64F linearInequalityConstraintsCMatrix, DenseMatrix64F linearInequalityConstraintsDVector, DenseMatrix64F solutionMatrix)
   {
      double maxSignedError = getMaxInequalityConstraintError(numberOfEqualityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solutionMatrix);
      assertTrue(maxSignedError < 1e-10);
   }

   private void verifyEqualityConstraintsDoNotHold(int numberOfEqualityConstraints, DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector, DenseMatrix64F solutionMatrix)
   {
      double maxAbsoluteError = getMaxEqualityConstraintError(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solutionMatrix);
      assertTrue(maxAbsoluteError > 1e-5);
   }

   private void verifyInequalityConstraintsDoNotHold(int numberOfInequalityConstraints, DenseMatrix64F linearInequalityConstraintsCMatrix, DenseMatrix64F linearInequalityConstraintsDVector, DenseMatrix64F solutionMatrix)
   {
      double maxError = getMaxInequalityConstraintError(numberOfInequalityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solutionMatrix);
      assertTrue(maxError > 1e-5);
   }

   private void verifyVariableBoundsHold(int testNumber, DenseMatrix64F variableLowerBounds, DenseMatrix64F variableUpperBounds, DenseMatrix64F solution)
   {
      for (int i = 0; i < variableLowerBounds.getNumRows(); i++)
      {
         assertTrue("In test number " + testNumber + " the solution " + solution.get(i, 0) + " is less than the lower bound " + variableLowerBounds.get(i, 0), solution.get(i, 0) >= variableLowerBounds.get(i, 0) - 1e-7);
      }

      for (int i = 0; i < variableUpperBounds.getNumRows(); i++)
      {
         assertTrue(solution.get(i, 0) <= variableUpperBounds.get(i, 0) + 1e-7);
      }
   }

   private double getMaxEqualityConstraintError(int numberOfEqualityConstraints, DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector, DenseMatrix64F solutionMatrix)
   {
      DenseMatrix64F checkMatrix = new DenseMatrix64F(numberOfEqualityConstraints, 1);
      CommonOps.mult(linearEqualityConstraintsAMatrix, solutionMatrix, checkMatrix);
      CommonOps.subtractEquals(checkMatrix, linearEqualityConstraintsBVector);

      return getMaxAbsoluteDataEntry(checkMatrix);
   }

   private double getMaxInequalityConstraintError(int numberOfInequalityConstraints, DenseMatrix64F linearInequalityConstraintsCMatrix, DenseMatrix64F linearInequalityConstraintsDVector, DenseMatrix64F solutionMatrix)
   {
      DenseMatrix64F checkMatrix = new DenseMatrix64F(numberOfInequalityConstraints, 1);
      CommonOps.mult(linearInequalityConstraintsCMatrix, solutionMatrix, checkMatrix);
      CommonOps.subtractEquals(checkMatrix, linearInequalityConstraintsDVector);

      return getMaxSignedDataEntry(checkMatrix);
   }

   private DenseMatrix64F projectOntoEqualityConstraints(DenseMatrix64F solutionMatrix, DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector)
   {
      int numberOfVariables = solutionMatrix.getNumRows();
      if (linearEqualityConstraintsAMatrix.getNumCols() != numberOfVariables)
         throw new RuntimeException();

      int numberOfConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      if (linearEqualityConstraintsBVector.getNumRows() != numberOfConstraints)
         throw new RuntimeException();

      DenseMatrix64F AZMinusB = new DenseMatrix64F(numberOfConstraints, 1);
      CommonOps.mult(linearEqualityConstraintsAMatrix, solutionMatrix, AZMinusB);
      CommonOps.subtractEquals(AZMinusB, linearEqualityConstraintsBVector);

      DenseMatrix64F AATransposeInverse = new DenseMatrix64F(numberOfConstraints, numberOfConstraints);
      DenseMatrix64F linearEqualityConstraintsAMatrixTranspose = new DenseMatrix64F(linearEqualityConstraintsAMatrix);
      CommonOps.transpose(linearEqualityConstraintsAMatrixTranspose);

      CommonOps.mult(linearEqualityConstraintsAMatrix, linearEqualityConstraintsAMatrixTranspose, AATransposeInverse);
      CommonOps.invert(AATransposeInverse);

      DenseMatrix64F ATransposeAATransposeInverse = new DenseMatrix64F(numberOfVariables, numberOfConstraints);
      CommonOps.mult(linearEqualityConstraintsAMatrixTranspose, AATransposeInverse, ATransposeAATransposeInverse);

      DenseMatrix64F vectorToSubtract = new DenseMatrix64F(numberOfVariables, 1);
      CommonOps.mult(ATransposeAATransposeInverse, AZMinusB, vectorToSubtract);

      DenseMatrix64F projectedSolutionMatrix = new DenseMatrix64F(solutionMatrix);
      CommonOps.subtractEquals(projectedSolutionMatrix, vectorToSubtract);

      return projectedSolutionMatrix;
   }

   private double getMaxAbsoluteDataEntry(DenseMatrix64F matrix)
   {
      int numberOfRows = matrix.getNumRows();
      int numberOfColumns = matrix.getNumCols();

      double max = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < numberOfRows; i++)
      {
         for (int j = 0; j < numberOfColumns; j++)
         {
            double absoluteValue = Math.abs(matrix.get(i, j));
            if (absoluteValue > max)
            {
               max = absoluteValue;
            }
         }
      }

      return max;
   }

   private double getMaxSignedDataEntry(DenseMatrix64F matrix)
   {
      int numberOfRows = matrix.getNumRows();
      int numberOfColumns = matrix.getNumCols();

      double max = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < numberOfRows; i++)
      {
         for (int j = 0; j < numberOfColumns; j++)
         {
            double value = matrix.get(i, j);
            if (value > max)
            {
               max = value;
            }
         }
      }

      return max;
   }
}
