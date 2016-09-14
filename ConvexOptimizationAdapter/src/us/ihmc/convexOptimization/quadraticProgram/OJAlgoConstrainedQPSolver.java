package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ojalgo.matrix.store.RawStore;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation.Result;
import org.ojalgo.optimisation.Optimisation.State;
import org.ojalgo.optimisation.convex.ConvexSolver;

import us.ihmc.tools.exceptions.NoConvergenceException;

public class OJAlgoConstrainedQPSolver extends ConstrainedQPSolver
{

   public OJAlgoConstrainedQPSolver()
   {

      ExpressionsBasedModel foo;
   }

   @Override
   public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F x,
         boolean initialize) throws NoConvergenceException
   {
      RawStore QDenseStore = new RawStore(Q.data, Q.getNumRows());

      double[][] fData = new double[f.getNumRows()][1];
      for (int i = 0; i < f.getNumRows(); i++)
      {
         fData[i][0] = -f.get(i, 0);
      }
      RawStore CDenseStore = new RawStore(fData);
      ConvexSolver.Builder builder = new ConvexSolver.Builder(QDenseStore, CDenseStore);

      if (Aeq != null)
      {
         RawStore AEDenseStore = new RawStore(Aeq.data, Aeq.getNumRows());
         RawStore BEDenseStore = new RawStore(beq.data, beq.getNumRows());
         builder.equalities(AEDenseStore, BEDenseStore);
      }

      if (Ain != null)
      {
         RawStore AIDenseStore = new RawStore(Ain.data, Ain.getNumRows());
         RawStore BIDenseStore = new RawStore(bin.data, bin.getNumRows());
         builder.inequalities(AIDenseStore, BIDenseStore);
      }

      ConvexSolver convexSolver = builder.build();

      Result result = convexSolver.solve();
      int size = result.size();

      for (int i=0; i<size; i++)
      {
         x.set(i, 0, result.doubleValue(i));
      }

      State state = result.getState();
      if (state.isFailure())
      {
         throw new NoConvergenceException();
      }

      return 1;
   }

   @Override
   public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F lb,
         DenseMatrix64F ub, DenseMatrix64F x, boolean initialize) throws NoConvergenceException
   {
      throw new RuntimeException("Implement me!");
   }

   @Override
   public boolean supportBoxConstraints()
   {
      return false;
   }

}
