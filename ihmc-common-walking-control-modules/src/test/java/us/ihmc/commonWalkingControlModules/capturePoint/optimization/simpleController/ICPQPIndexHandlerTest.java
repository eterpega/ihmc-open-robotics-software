package us.ihmc.commonWalkingControlModules.capturePoint.optimization.simpleController;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput.ICPQPIndexHandler;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ICPQPIndexHandlerTest
{
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testRegisterFootstep()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler();
      indexHandler.registerFootstep();

      Assert.assertTrue(indexHandler.useStepAdjustment());
      Assert.assertEquals(1, indexHandler.getNumberOfFootstepsToConsider());
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testSizing()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler();

      Assert.assertEquals(0, indexHandler.getFeedbackCMPIndex());
      Assert.assertFalse(indexHandler.useAngularMomentum());
      Assert.assertFalse(indexHandler.useStepAdjustment());

      indexHandler.computeProblemSize();

      Assert.assertEquals(0, indexHandler.getFeedbackCMPIndex());
      Assert.assertEquals(2, indexHandler.getAngularMomentumIndex());
      Assert.assertEquals(2, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(2, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(2, indexHandler.getNumberOfFreeVariables());
      Assert.assertFalse(indexHandler.useAngularMomentum());
      Assert.assertFalse(indexHandler.useStepAdjustment());

      indexHandler.resetFootsteps();

      Assert.assertEquals(0, indexHandler.getFeedbackCMPIndex());
      Assert.assertEquals(2, indexHandler.getAngularMomentumIndex());
      Assert.assertEquals(2, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(2, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(2, indexHandler.getNumberOfFreeVariables());
      Assert.assertFalse(indexHandler.useAngularMomentum());
      Assert.assertFalse(indexHandler.useStepAdjustment());

      indexHandler.registerFootstep();

      Assert.assertEquals(0, indexHandler.getFeedbackCMPIndex());
      Assert.assertEquals(2, indexHandler.getAngularMomentumIndex());
      Assert.assertEquals(2, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(2, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(1, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(2, indexHandler.getNumberOfFreeVariables());
      Assert.assertFalse(indexHandler.useAngularMomentum());
      Assert.assertTrue(indexHandler.useStepAdjustment());

      indexHandler.computeProblemSize();

      Assert.assertEquals(0, indexHandler.getFeedbackCMPIndex());
      Assert.assertEquals(2, indexHandler.getAngularMomentumIndex());
      Assert.assertEquals(2, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(2, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(2, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(1, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(4, indexHandler.getNumberOfFreeVariables());
      Assert.assertFalse(indexHandler.useAngularMomentum());
      Assert.assertTrue(indexHandler.useStepAdjustment());

      indexHandler.resetFootsteps();

      Assert.assertEquals(0, indexHandler.getFeedbackCMPIndex());
      Assert.assertEquals(2, indexHandler.getAngularMomentumIndex());
      Assert.assertEquals(2, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(2, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(2, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(4, indexHandler.getNumberOfFreeVariables());
      Assert.assertFalse(indexHandler.useAngularMomentum());
      Assert.assertFalse(indexHandler.useStepAdjustment());

      indexHandler.setUseAngularMomentum(true);

      Assert.assertEquals(0, indexHandler.getFeedbackCMPIndex());
      Assert.assertEquals(2, indexHandler.getAngularMomentumIndex());
      Assert.assertEquals(2, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(2, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(2, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(4, indexHandler.getNumberOfFreeVariables());
      Assert.assertTrue(indexHandler.useAngularMomentum());
      Assert.assertFalse(indexHandler.useStepAdjustment());

      indexHandler.computeProblemSize();

      Assert.assertEquals(0, indexHandler.getFeedbackCMPIndex());
      Assert.assertEquals(2, indexHandler.getAngularMomentumIndex());
      Assert.assertEquals(4, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(4, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(4, indexHandler.getNumberOfFreeVariables());
      Assert.assertTrue(indexHandler.useAngularMomentum());
      Assert.assertFalse(indexHandler.useStepAdjustment());

      indexHandler.resetFootsteps();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      Assert.assertEquals(0, indexHandler.getFeedbackCMPIndex());
      Assert.assertEquals(2, indexHandler.getAngularMomentumIndex());
      Assert.assertEquals(4, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(4, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(2, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(1, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(6, indexHandler.getNumberOfFreeVariables());
      Assert.assertTrue(indexHandler.useAngularMomentum());
      Assert.assertTrue(indexHandler.useStepAdjustment());

      indexHandler.resetFootsteps();
      indexHandler.setUseAngularMomentum(false);
      indexHandler.registerFootstep();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      Assert.assertEquals(0, indexHandler.getFeedbackCMPIndex());
      Assert.assertEquals(2, indexHandler.getAngularMomentumIndex());
      Assert.assertEquals(2, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(2, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(4, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(2, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(6, indexHandler.getNumberOfFreeVariables());
      Assert.assertFalse(indexHandler.useAngularMomentum());
      Assert.assertTrue(indexHandler.useStepAdjustment());

      indexHandler.resetFootsteps();
      indexHandler.setUseAngularMomentum(true);
      indexHandler.registerFootstep();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      Assert.assertEquals(0, indexHandler.getFeedbackCMPIndex());
      Assert.assertEquals(2, indexHandler.getAngularMomentumIndex());
      Assert.assertEquals(4, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(4, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(4, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(2, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(8, indexHandler.getNumberOfFreeVariables());
      Assert.assertTrue(indexHandler.useAngularMomentum());
      Assert.assertTrue(indexHandler.useStepAdjustment());
   }
}
