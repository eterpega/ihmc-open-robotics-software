package us.ihmc.convexOptimization.quadraticProgram;

import javax.print.attribute.standard.NumberOfInterveningJobs;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.convexOptimization.QuadProgWrapper;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class QuadProgSolver extends ConstrainedQPSolver
{

   /*
    * The problem is in the form: min 0.5 * x G x + g0 x s.t. CE^T x + ce0 = 0
    * CI^T x + ci0 >= 0
    */

   private final QuadProgWrapper qpWrapper;
   private final DenseMatrix64F negativeAEqualityConstraints = new DenseMatrix64F(0, 0), negativeAInequalityConstraints = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F bEqualityConstraints = new DenseMatrix64F(0, 0), bInequalityConstraints = new DenseMatrix64F(0, 0);

   public QuadProgSolver()
   {
      this(1, 0, 0);
   }

   public QuadProgSolver(int numberOfVariables, int numberOfEqualityConstraints, int numberOfInequalityConstraints)
   {
      qpWrapper = new QuadProgWrapper(numberOfVariables, numberOfEqualityConstraints, numberOfInequalityConstraints);
      allocateTempraryMatrixOnDemand(numberOfVariables, numberOfEqualityConstraints, numberOfInequalityConstraints);
   }

   private void allocateTempraryMatrixOnDemand(int numberOfVariables, int numberOfEqualityConstraints, int numberOfInequalityConstraints)
   {
      negativeAEqualityConstraints.reshape(numberOfVariables, numberOfEqualityConstraints);
      bEqualityConstraints.reshape(numberOfEqualityConstraints, 1);
      negativeAInequalityConstraints.reshape(numberOfVariables, numberOfInequalityConstraints);
      bInequalityConstraints.reshape(numberOfInequalityConstraints, 1);
   }

   @Override
   public boolean supportBoxConstraints()
   {
      return false;
   }

   @Override
   public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F x, boolean initialize) throws NoConvergenceException
   {
      return solve(Q, f, Aeq, beq, Ain, bin, null, null, x, initialize);
   }

   @Override
   public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F lb, DenseMatrix64F ub, DenseMatrix64F x, boolean initialize)
         throws NoConvergenceException
   {
      int numberOfVariables = Q.getNumRows();
      int numberOfEqualityConstraints = 0;
      if (Aeq != null)
      {
         numberOfEqualityConstraints = Aeq.getNumRows();
      }

      int numberOfInequalityConstraints = 0;

      if (Ain != null)
      {
         numberOfInequalityConstraints = Ain.getNumRows();
      }

      allocateTempraryMatrixOnDemand(numberOfVariables, numberOfEqualityConstraints, numberOfInequalityConstraints);

      if (Aeq != null)
      {
         CommonOps.transpose(Aeq, this.negativeAEqualityConstraints);
         CommonOps.scale(-1.0, this.negativeAEqualityConstraints);
         bEqualityConstraints.set(beq);
      }

      if (Ain != null)
      {
         CommonOps.transpose(Ain, this.negativeAInequalityConstraints);
         CommonOps.scale(-1.0, this.negativeAInequalityConstraints);
         bInequalityConstraints.set(bin);
      }

      return qpWrapper.solve(Q, f, negativeAEqualityConstraints, bEqualityConstraints, negativeAInequalityConstraints, bInequalityConstraints, x, initialize);
   }
}
