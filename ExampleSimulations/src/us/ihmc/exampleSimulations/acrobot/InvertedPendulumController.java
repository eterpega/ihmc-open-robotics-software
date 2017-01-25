package us.ihmc.exampleSimulations.acrobot;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.robotController.SimpleRobotController;

public class InvertedPendulumController extends SimpleRobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final InvertedPendulumRobot robot;

   private final YoFramePoint footViz = new YoFramePoint("FootLocation", worldFrame, registry);
   private final YoFrameVector simulatedReactionViz = new YoFrameVector("GroundReactionSimulated", worldFrame, registry);
   private final YoFrameVector computedReactionViz = new YoFrameVector("GroundReactionComputed", worldFrame, registry);

   private final double m1;
   private final double I1;
   private final double l1;
   private final double g;

   public InvertedPendulumController(InvertedPendulumRobot robot, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.robot = robot;

      YoGraphicVector simulatedGRFYoGraphic = new YoGraphicVector("simulatedGRF", footViz, simulatedReactionViz, 0.05, YoAppearance.DarkGreen(), true);
      graphicsListRegistry.registerYoGraphic("Simulated GRF", simulatedGRFYoGraphic);
      YoGraphicVector computedGRFYoGraphic = new YoGraphicVector("computedGRF", footViz, computedReactionViz, 0.05, YoAppearance.Red(), true);
      graphicsListRegistry.registerYoGraphic("Computed GRF", computedGRFYoGraphic);

      m1 = InvertedPendulumRobot.MASS;
      I1 = InvertedPendulumRobot.INERTIA;
      l1 = InvertedPendulumRobot.LENGTH;
      g = -robot.getGravityZ();
   }

   @Override
   public void doControl()
   {
      double theta1 = robot.getTheta1();
      double theta1d = robot.getTheta1d();

      // no inertia:
//      double xGRF = m1 * (thetad * thetad * l1 - 2.0 * g * Math.cos(theta)) * Math.sin(theta) / 2.0;
//      double zGRF = m1 * (thetad * thetad * l1 - 2.0 * g * Math.cos(theta)) * Math.cos(theta) / 2.0;

      double t1 = 4.0 * I1 + l1 * l1 * m1;
      double xGRF = l1 * m1 * (theta1d * theta1d * t1 - 2.0 * g * l1 * m1 * Math.cos(theta1)) * Math.sin(theta1) / (2.0 * t1);
      double zGRF = m1 * (-2.0 * g * t1 + l1 * (theta1d * theta1d * t1 * Math.cos(theta1) + 2.0 * g * l1 * m1 * Math.sin(theta1) * Math.sin(theta1))) / (2.0 * t1);

      Vector3d computedForce = new Vector3d(xGRF, 0.0, zGRF);
      computedForce.negate();

      footViz.set(robot.getContactPosition());
      simulatedReactionViz.set(robot.getContactForce());
      computedReactionViz.set(computedForce);
   }

}
