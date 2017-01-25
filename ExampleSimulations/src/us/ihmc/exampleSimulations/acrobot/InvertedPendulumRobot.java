package us.ihmc.exampleSimulations.acrobot;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class InvertedPendulumRobot extends Robot
{
   private static final boolean SHOW_MASS_ELIPSOIDS = false;
   private static final boolean SHOW_COORDINATE_SYSTEMS = false;

   public static final double MASS = 2.0;
   public static final double INERTIA = 0.1;
   public static final double LENGTH = 1.0;

   private final FloatingPlanarJoint root;
   private final GroundContactPoint contactPoint;

   public InvertedPendulumRobot()
   {
      super("pendulumRobot");

      root = new FloatingPlanarJoint("base", new Vector3d(), this);
      root.setLink(pendulumLink());
      this.addRootJoint(root);

      contactPoint = new GroundContactPoint("gc1", new Vector3d(0.0, 0.0, 0.0), this);
      root.addGroundContactPoint(contactPoint);

      GroundContactModel groundModel = new LinearGroundContactModel(this, 1500.0, 150.0, 50.0, 1000.0, this.getRobotsYoVariableRegistry());
      groundModel.setGroundProfile3D(new FlatGroundProfile());
      this.setGroundContactModel(groundModel);
   }

   private Link pendulumLink()
   {
      Link link = new Link("link");
      link.setMass(MASS);
      link.setComOffset(0.0, 0.0, LENGTH / 2.0);
      link.setMomentOfInertia(0.001, INERTIA, 0.001);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, 0.0);
      linkGraphics.addCylinder(LENGTH, 0.025, YoAppearance.Blue());
      link.setLinkGraphics(linkGraphics);

      if (SHOW_MASS_ELIPSOIDS)
         link.addEllipsoidFromMassProperties();
      if (SHOW_COORDINATE_SYSTEMS)
         link.addCoordinateSystemToCOM(0.3);

      return link;
   }

   public Vector3d getContactForce()
   {
      Vector3d ret = new Vector3d();
      contactPoint.getForce(ret);
      return ret;
   }

   public Point3d getContactPosition()
   {
      Point3d ret = new Point3d();
      contactPoint.getPosition(ret);
      return ret;
   }

   public double getTheta1()
   {
      return root.getQ_rot().getDoubleValue();
   }

   public double getTheta1d()
   {
      return root.getQd_rot().getDoubleValue();
   }

   public double getTheta1dd()
   {
      return root.getQdd_rot().getDoubleValue();
   }

   public void setTheta1(double theta1)
   {
      root.setRotation(theta1);
   }

   public void setTheta1d(double theta1d)
   {
      root.setRotationalVelocity(theta1d);
   }
}
