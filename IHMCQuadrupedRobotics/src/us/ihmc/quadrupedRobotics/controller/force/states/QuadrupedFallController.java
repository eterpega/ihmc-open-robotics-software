package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.SoleWaypoint;
import us.ihmc.quadrupedRobotics.providers.QuadrupedSoleWaypointInputProvider;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;

import javax.vecmath.Vector3d;
import java.util.ArrayList;

public class QuadrupedFallController extends QuadrupedSoleWaypointController
{
   //todo:add to registry correctly
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + "2");

   // Parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(QuadrupedFallController.class, registry);
   private final DoubleParameter trajectoryTimeParameter = parameterFactory.createDouble("trajectoryTime", 3.0);
   private final DoubleParameter stanceLengthParameter = parameterFactory.createDouble("stanceLength", 1.0);
   private final DoubleParameter stanceWidthParameter = parameterFactory.createDouble("stanceWidth", 0.8);
   private final DoubleParameter stanceHeightParameter = parameterFactory.createDouble("stanceHeight", 0.40);
   private final DoubleParameter stanceXOffsetParameter = parameterFactory.createDouble("stanceXOffset", 0.05);
   private final DoubleParameter stanceYOffsetParameter = parameterFactory.createDouble("stanceYOffset", 0.0);

   private ArrayList<SoleWaypoint> solewaypoints = new ArrayList<>();
   private FramePoint finalSolePosition = new FramePoint();
   private final Vector3d zeroVelocity = new Vector3d(0, 0, 0);

   public enum fallBehaviors
   {
      FREEZE, GO_HOME_Z, GO_HOME_XYZ
   }

   private final EnumYoVariable<fallBehaviors> fallBehavior = EnumYoVariable.create("fallBehaviors", fallBehaviors.class, registry);

   public QuadrupedFallController(QuadrupedRuntimeEnvironment environment, QuadrupedForceControllerToolbox controllerToolbox,
         QuadrupedSoleWaypointInputProvider inputProvider)
   {
      super(environment, controllerToolbox, inputProvider);
      fallBehavior.set(fallBehaviors.FREEZE);
      environment.getParentRegistry().addChild(registry);
   }

   @Override
   public void onEntry()
   {
      super.onEntry();
      getSoleWaypointInputProvider().get().clear();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solewaypoints = new ArrayList<>();
         FramePoint initialSolePosition = getTaskSpaceEstimates().getSolePosition(quadrant);
         initialSolePosition.changeFrame(getReferenceFrames().getBodyFrame());
         solewaypoints.add(new SoleWaypoint(initialSolePosition.getPoint(), zeroVelocity, 0.0));
         finalSolePosition = new FramePoint();
         computeFinalSolePosition(quadrant, finalSolePosition);
         solewaypoints.add(new SoleWaypoint(finalSolePosition.getPoint(), zeroVelocity, trajectoryTimeParameter.get()));
         getSoleWaypointInputProvider().get().set(quadrant, solewaypoints);
      }
      createSoleWaypointTrajectory(getSoleWaypointInputProvider());
   }

   private void computeFinalSolePosition(RobotQuadrant quadrant, FramePoint finalSolePosition)
   {
      finalSolePosition.setToZero(getReferenceFrames().getBodyFrame());
      switch (fallBehavior.getEnumValue())
      {
      case GO_HOME_XYZ:
         finalSolePosition.add(quadrant.getEnd().negateIfHindEnd(stanceLengthParameter.get() / 2.0), 0.0, 0.0);
         finalSolePosition.add(0.0, quadrant.getSide().negateIfRightSide(stanceWidthParameter.get() / 2.0), 0.0);
         finalSolePosition.add(stanceXOffsetParameter.get(), stanceYOffsetParameter.get(), -stanceHeightParameter.get());
         break;
      case GO_HOME_Z:
         finalSolePosition.add(getTaskSpaceEstimates().getSolePosition(quadrant).getX(), getTaskSpaceEstimates().getSolePosition(quadrant).getY(),
               -stanceHeightParameter.get());
         break;
      default:
         finalSolePosition.add(getTaskSpaceEstimates().getSolePosition(quadrant).getX(), getTaskSpaceEstimates().getSolePosition(quadrant).getY(),
               getTaskSpaceEstimates().getSolePosition(quadrant).getZ());
         break;
      }
   }

   @Override
   public void onExit()
   {
   }
}
