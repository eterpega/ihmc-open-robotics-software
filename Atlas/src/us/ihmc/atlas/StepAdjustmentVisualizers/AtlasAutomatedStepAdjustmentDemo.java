package us.ihmc.atlas.StepAdjustmentVisualizers;

import jxl.Workbook;
import jxl.write.*;
import jxl.write.Number;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.controllers.ControllerFailureException;

import java.io.File;
import java.io.IOException;

public class AtlasAutomatedStepAdjustmentDemo
{
   private static final WritableCellFormat defaultFormat = new WritableCellFormat();
   private static final WritableCellFormat defaultNumberFormat = new WritableCellFormat(NumberFormats.FLOAT);

   private static double initialPushPercent = 1.5;
   private static double pushIncrement = 0.01;

   private static boolean showGui = false;

   public AtlasAutomatedStepAdjustmentDemo()
   {
   }

   public double computeMaxPushPercent(StepScriptType stepScriptType, TestType testType, PushDirection pushDirection)
   {
      boolean successful = true;
      boolean previousSuccessful = false;

      boolean failedOnce = false;
      boolean passedOnce = false;

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
            passedOnce = true;
         }
         catch (ControllerFailureException e)
         {
            successful = false;
            failedOnce = true;
         }
         catch (Exception e)
         { }

         demo.destroySimulation();
         demo = null;


         if (!successful && previousSuccessful && failedOnce && passedOnce)
            executeAgain = false;
         else
            executeAgain = true;

         if (failedOnce && passedOnce)
            previousSuccessful = successful;

         if (executeAgain)
         {
            double increment = pushIncrement;
            if (!failedOnce || !passedOnce)
               increment = 10.0 * pushIncrement;

            if (successful)
               pushPercent += increment;
            else
               pushPercent -= increment;
         }
         else
         {
            pushPercent -= pushIncrement;
         }
      }

      PrintTools.info("\n\nThe maximum recoverable push for step type " + stepScriptType + ", test type " + testType + ", and push direction " +
                            pushDirection + " is weight percent " + pushPercent + ".\n\n");

      return pushPercent;
   }


   private static WritableWorkbook createWorkbook(File workbookFile)
   {
      WritableWorkbook writableWorkBook = null;
      try
      {
         writableWorkBook = Workbook.createWorkbook(workbookFile);
      }
      catch (IOException ex)
      {
         PrintTools.error("Failed to open Excel workbook. " + workbookFile);
      }

      return writableWorkBook;
   }

   private static void addStringToSheet(WritableSheet dataSheet, int column, int row, String string)
   {
      addStringToSheet(dataSheet, column, row, string, defaultFormat);
   }

   private static void addStringToSheet(WritableSheet dataSheet, int column, int row, String string, WritableCellFormat format)
   {
      Label cell = new Label(column, row, string, format);
      addCell(dataSheet, cell);
   }

   private static void addNumberToSheet(WritableSheet dataSheet, int column, int row, double value)
   {
      addNumberToSheet(dataSheet, column, row, value, defaultNumberFormat);
   }

   private static void addNumberToSheet(WritableSheet dataSheet, int column, int row, double value, WritableCellFormat format)
   {
      Number cell = new Number(column, row, value, format);
      addCell(dataSheet, cell);
   }

   private static void addCell(WritableSheet dataSheet, WritableCell cell)
   {
      try
      {
         dataSheet.addCell(cell);
      }
      catch (WriteException e)
      {
         e.printStackTrace();
      }
   }

   private static void addPushDirectionHeaders(WritableSheet sheet)
   {
      int colNumber = 1;
      for (PushDirection pushDirection : PushDirection.values())
      {
         addStringToSheet(sheet, colNumber, 0, pushDirection.toString());
         colNumber++;
      }
   }

   private static void addControlTypeRowName(WritableSheet sheet)
   {
      addStringToSheet(sheet, 0, 0, "Control Test Type");
      int rowNumber = 1;
      for (TestType testType : TestType.values())
      {
         addStringToSheet(sheet, 0, rowNumber, testType.toString());
         rowNumber++;
      }
   }

   private static void saveWorkbook(WritableWorkbook workbook, File workbookFile)
   {
      if (workbook != null)
      {
         // save and close Excel workbook
         try
         {
            workbook.write();
            workbook.close();
            System.out.println("Done creating Excel workbook");
         }
         catch (Exception ex)
         {
            PrintTools.error("Trouble saving Excel workbook " + workbookFile.getAbsolutePath());
         }
      }
   }

   public static void main(String[] args)
   {
      StepScriptType stepScriptType = StepScriptType.FORWARD_FAST;
      File forwardFastFile = new File("/home/robert/StepAdjustmentForwardFast.xls");
      WritableWorkbook forwardFastWorkbook = createWorkbook(forwardFastFile);
      WritableSheet forwardFastSheet = forwardFastWorkbook.createSheet(stepScriptType.toString(), forwardFastWorkbook.getNumberOfSheets());

      // setup name headers
      addPushDirectionHeaders(forwardFastSheet);
      addControlTypeRowName(forwardFastSheet);

      int colNumber = 1;
      for (PushDirection pushDirection : PushDirection.values())
      {
         int rowNumber = 1;
         for (TestType testType : TestType.values())
         {
            AtlasAutomatedStepAdjustmentDemo demo = new AtlasAutomatedStepAdjustmentDemo();
            double pushPercent = demo.computeMaxPushPercent(stepScriptType, testType, pushDirection);
            addNumberToSheet(forwardFastSheet, colNumber, rowNumber, pushPercent);

            rowNumber++;
         }
         colNumber++;
      }
      saveWorkbook(forwardFastWorkbook, forwardFastFile);




      stepScriptType = StepScriptType.FORWARD_SLOW;
      File forwardSlowFile = new File("/home/robert/StepAdjustmentForwardSlow.xls");
      WritableWorkbook forwardSlowWorkbook = createWorkbook(forwardSlowFile);
      WritableSheet forwardSlowSheet = forwardSlowWorkbook.createSheet(stepScriptType.toString(), forwardSlowWorkbook.getNumberOfSheets());

      // setup name headers
      addPushDirectionHeaders(forwardSlowSheet);
      addControlTypeRowName(forwardSlowSheet);

      colNumber = 1;
      for (PushDirection pushDirection : PushDirection.values())
      {
         int rowNumber = 1;
         for (TestType testType : TestType.values())
         {
            AtlasAutomatedStepAdjustmentDemo demo = new AtlasAutomatedStepAdjustmentDemo();
            double pushPercent = demo.computeMaxPushPercent(stepScriptType, testType, pushDirection);
            addNumberToSheet(forwardSlowSheet, colNumber, rowNumber, pushPercent);

            rowNumber++;
         }
         colNumber++;
      }
      saveWorkbook(forwardSlowWorkbook, forwardSlowFile);




      stepScriptType = StepScriptType.STATIONARY_FAST;
      File stationaryFastFile = new File("/home/robert/StepAdjustmentStationaryFast.xls");
      WritableWorkbook stationaryFastWorkbook = createWorkbook(stationaryFastFile);
      WritableSheet stationaryFastSheet = stationaryFastWorkbook.createSheet(stepScriptType.toString(), stationaryFastWorkbook.getNumberOfSheets());

      // setup name headers
      addPushDirectionHeaders(stationaryFastSheet);
      addControlTypeRowName(stationaryFastSheet);

      colNumber = 1;
      for (PushDirection pushDirection : PushDirection.values())
      {
         int rowNumber = 1;
         for (TestType testType : TestType.values())
         {
            AtlasAutomatedStepAdjustmentDemo demo = new AtlasAutomatedStepAdjustmentDemo();
            double pushPercent = demo.computeMaxPushPercent(stepScriptType, testType, pushDirection);
            addNumberToSheet(stationaryFastSheet, colNumber, rowNumber, pushPercent);

            rowNumber++;
         }
         colNumber++;
      }
      saveWorkbook(stationaryFastWorkbook, stationaryFastFile);

      stepScriptType = StepScriptType.STATIONARY_SLOW;
      File stationarySlowFile = new File("/home/robert/StepAdjustmentStationarySlow.xls");
      WritableWorkbook stationarySlowWorkbook = createWorkbook(stationarySlowFile);
      WritableSheet stationarySlowSheet = stationarySlowWorkbook.createSheet(stepScriptType.toString(), stationarySlowWorkbook.getNumberOfSheets());

      // setup name headers
      addPushDirectionHeaders(stationarySlowSheet);
      addControlTypeRowName(stationarySlowSheet);

      colNumber = 1;
      for (PushDirection pushDirection : PushDirection.values())
      {
         int rowNumber = 1;
         for (TestType testType : TestType.values())
         {
            AtlasAutomatedStepAdjustmentDemo demo = new AtlasAutomatedStepAdjustmentDemo();
            double pushPercent = demo.computeMaxPushPercent(stepScriptType, testType, pushDirection);
            addNumberToSheet(stationarySlowSheet, colNumber, rowNumber, pushPercent);

            rowNumber++;
         }
         colNumber++;
      }
      saveWorkbook(stationarySlowWorkbook, stationarySlowFile);

   }
}