package us.ihmc.quadrupedRobotics.planning.stepStream;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.PreallocatedList;

public interface QuadrupedStepStream
{
   /**
    * Initialize the step stream.
    */
   void onEntry();

   /**
    * Update the step queue and body orientation at each time step.
    */
   void process();

   /**
    * Cleanup.
    */
   void onExit();

   /**
    * Get the queue of ongoing and upcoming steps.
    * @return timed step queue
    */
   PreallocatedList<? extends QuadrupedTimedStep> getSteps();

   /**
    * Get the nominal body orientation at the current time step.
    */
   void getBodyOrientation(FrameQuaternion bodyOrientation);

   FrameQuaternionReadOnly getBodyOrientation();
}
