package us.ihmc.footstepPlanning;

public enum FootstepPlanningResult
{
   OPTIMAL_SOLUTION,
   SUB_OPTIMAL_SOLUTION,
   TIMED_OUT_BEFORE_SOLUTION,
   NO_PATH_EXISTS,
   SNAPPING_FAILED,
   PLANNER_FAILED;

   public static final FootstepPlanningResult[] values = values();

   public boolean validForExecution()
   {
      switch (this)
      {
      case OPTIMAL_SOLUTION:
      case SUB_OPTIMAL_SOLUTION:
         return true;
      default:
         return false;
      }
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static FootstepPlanningResult fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
