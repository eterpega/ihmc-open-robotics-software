package us.ihmc.quadrupedRobotics.controller.force;

public enum QuadrupedForceControllerState
{
   JOINT_INITIALIZATION,
   DO_NOTHING,
   STAND_PREP,
   STAND_READY,
   STAND,
   STEP,
   TROT,
   PACE,
   XGAIT,
   FALL,
   JOINT_POSE,
   CARTESIAN_SOLE
}