package us.ihmc.convexOptimization.quadraticProgram;

import java.util.Arrays;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class CompositeActiveSetQPSolver extends ConstrainedQPSolver
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final SimpleActiveSetQPStandaloneSolver solver = new SimpleActiveSetQPStandaloneSolver(10);
   private final ConstrainedQPSolver fullSolver = new QuadProgSolver();
   //   ConstrainedQPSolver fullSolver = new OASESConstrainedQPSolver(registry);
   private boolean[] linearInequalityActiveSet;

   private final LongYoVariable fullSolverCount = new LongYoVariable("fullSolverCount", registry);
   private final LongYoVariable simpleSolverIterations = new LongYoVariable("simpleSolverIterations", registry);

   public CompositeActiveSetQPSolver(YoVariableRegistry parentRegistry)
   {
      //parentRegistry.addChild(registry);
   }

   @Override
   public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F x,
         boolean initialize) throws NoConvergenceException
   {
      //allocate on demand

      if (Aeq == null)
      {
         Aeq = new DenseMatrix64F(0, Q.getNumRows());
         beq = new DenseMatrix64F(0, 1);
      }

      if (Ain == null)
      {
         Ain = new DenseMatrix64F(0, Q.getNumRows());
         Ain = new DenseMatrix64F(0, 1);
      }

      if (linearInequalityActiveSet == null)
         linearInequalityActiveSet = new boolean[Ain.numRows];
      else if (linearInequalityActiveSet.length != Ain.numRows)
      {
         System.err.println("linearInequalitySize changes, cold start with empty set");
         linearInequalityActiveSet = new boolean[Ain.numRows];
      }



      if (initialize)
         Arrays.fill(linearInequalityActiveSet, false);
      int iter = solver.solve(Q, f, Aeq, beq, Ain, bin, linearInequalityActiveSet, x);
      simpleSolverIterations.set(iter);

      if (iter < 0)
      {
         fullSolverCount.increment();
         Arrays.fill(linearInequalityActiveSet, false);
         fullSolver.solve(Q, f, Aeq, beq, Ain, bin, x, initialize);
      }

      return iter;

   }

   @Override
   public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F lb,
         DenseMatrix64F ub, DenseMatrix64F x, boolean initialize) throws NoConvergenceException
   {
      throw new RuntimeException("Not Implemented");
   }

   @Override
   public boolean supportBoxConstraints()
   {
      return false;
   }

}
