package us.ihmc.atlas.StepAdjustmentVisualizers;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.controllers.ControllerFailureException;

public class AtlasAutomatedStepAdjustmentDemo
{
   private static StepScriptType stepScriptType = StepScriptType.FORWARD_SLOW;
   private static TestType testType = TestType.BIG_ADJUSTMENT;
   private static PushDirection pushDirection = PushDirection.BACKWARD_IN_30;

   private static double initialPushPercent = 0.45;
   private static double pushIncrement = 0.05;

   private static boolean showGui = false;

   public AtlasAutomatedStepAdjustmentDemo(StepScriptType stepScriptType, TestType testType, PushDirection pushDirection)
   {
      boolean successful = true;
      boolean previousSuccessful = false;

      boolean executeAgain = true;

      double pushPercent = initialPushPercent;

      double simulationTime;
      switch (stepScriptType)
      {
      case FORWARD_FAST:
         simulationTime = 6.0;
         break;
      case FORWARD_SLOW:
         simulationTime = 10.0;
         break;
      case STATIONARY_FAST:
         simulationTime = 6.5;
         break;
      default:
         simulationTime = 9.5;
      }

      while (executeAgain)
      {

         AtlasStepAdjustmentDemo demo = new AtlasStepAdjustmentDemo(stepScriptType, testType, pushDirection, pushPercent, showGui);

         successful = true;
         try
         {
            demo.simulateAndBlock(simulationTime);
         }
         catch (ControllerFailureException e)
         {
            successful = false;
         }
         catch (Exception e)
         { }

         demo.destroySimulation();
         demo = null;


         if (!successful && previousSuccessful)
            executeAgain = false;
         else
            executeAgain = true;

         if (successful)
            previousSuccessful = true;

         if (executeAgain)
         {
            if (successful)
               pushPercent += pushIncrement;
            else
               pushPercent -= pushIncrement;
         }
         else
         {
            pushPercent -= pushIncrement;
         }
      }

      PrintTools.info("\n\nThe maximum recoverable push for step type " + stepScriptType + ", test type " + testType + ", and push direction " +
                            pushDirection + " is weight percent " + pushPercent + ".\n\n");
   }


   public static void main(String[] args)
   {
      for (StepScriptType stepScriptType : StepScriptType.values())
      {
         AtlasAutomatedStepAdjustmentDemo demo = new AtlasAutomatedStepAdjustmentDemo(stepScriptType, testType, pushDirection);
      }
   }
}