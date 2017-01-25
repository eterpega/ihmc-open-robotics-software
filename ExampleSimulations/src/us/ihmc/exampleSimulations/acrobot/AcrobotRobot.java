package us.ihmc.exampleSimulations.acrobot;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class AcrobotRobot extends Robot
{
   private static final boolean FIX_TO_WORLD = true;

   private static final boolean SHOW_MASS_ELIPSOIDS = false;
   private static final boolean SHOW_COORDINATE_SYSTEMS = false;

   public static final double LEG_MASS = 2.0;
   public static final double LEG_INERTIA = 0.00001;
   public static final double LEG_LENGTH = 0.5;

   public static final double BODY_MASS = 3.0;
   public static final double BODY_INERTIA = 0.00001;
   public static final double BODY_LENGTH = 1.0;

   private static final double DUMMY_MASS = 100.0;

   private final PinJoint ankle;
   private final PinJoint hip;
   private final SliderJoint xSlider;
   private final SliderJoint zSlider;

   public AcrobotRobot()
   {
      super("Acrobot");

      ankle = new PinJoint("ankle", new Vector3d(), this, Axis.Y);
      ankle.setLink(legLink());

      if (FIX_TO_WORLD)
      {
         this.addRootJoint(ankle);
         xSlider = null;
         zSlider = null;
      }
      else
      {
         xSlider = new SliderJoint("xSlider", new Vector3d(), this, Axis.X);
         xSlider.setLink(dummyLink());
         this.addRootJoint(xSlider);

         zSlider = new SliderJoint("zSlider", new Vector3d(), this, Axis.Z);
         zSlider.setLink(dummyLink());
         xSlider.addJoint(zSlider);

         zSlider.addJoint(ankle);
      }

      hip = new PinJoint("hip", new Vector3d(0.0, 0.0, LEG_LENGTH), this, Axis.Y);
      hip.setLink(bodyLink());
      ankle.addJoint(hip);

      GroundContactModel groundModel = new LinearGroundContactModel(this, 1500.0, 150.0, 50.0, 1000.0, this.getRobotsYoVariableRegistry());
      groundModel.setGroundProfile3D(new FlatGroundProfile());
      this.setGroundContactModel(groundModel);

      setGRFz(0.0);
   }

   private Link dummyLink()
   {
      Link link = new Link("dummyLink");
      link.setMass(DUMMY_MASS);
      link.setMomentOfInertia(1.0E10, 1.0E10, 1.0E10);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(0.05, YoAppearance.White());
      link.setLinkGraphics(linkGraphics);

      if (SHOW_MASS_ELIPSOIDS)
         link.addEllipsoidFromMassProperties();
      if (SHOW_COORDINATE_SYSTEMS)
         link.addCoordinateSystemToCOM(0.3);

      return link;
   }

   private Link legLink()
   {
      Link link = new Link("link");
      link.setMass(LEG_MASS);
      link.setComOffset(0.0, 0.0, LEG_LENGTH / 2.0);
      link.setMomentOfInertia(0.001, LEG_INERTIA, 0.001);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, 0.0);
      linkGraphics.addCylinder(LEG_LENGTH, 0.025, YoAppearance.Blue());
      link.setLinkGraphics(linkGraphics);

      if (SHOW_MASS_ELIPSOIDS)
         link.addEllipsoidFromMassProperties();
      if (SHOW_COORDINATE_SYSTEMS)
         link.addCoordinateSystemToCOM(0.3);

      return link;
   }

   private Link bodyLink()
   {
      Link link = new Link("link");
      link.setMass(BODY_MASS);
      link.setComOffset(0.0, 0.0, BODY_LENGTH / 2.0);
      link.setMomentOfInertia(0.001, BODY_INERTIA, 0.001);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, 0.0);
      linkGraphics.addCylinder(BODY_LENGTH, 0.025, YoAppearance.Blue());
      link.setLinkGraphics(linkGraphics);

      if (SHOW_MASS_ELIPSOIDS)
         link.addEllipsoidFromMassProperties();
      if (SHOW_COORDINATE_SYSTEMS)
         link.addCoordinateSystemToCOM(0.3);

      return link;
   }

   public Vector3d getContactForce()
   {
      if (zSlider == null || xSlider == null)
         return new Vector3d();
      return new Vector3d(xSlider.getTau(), 0.0, zSlider.getTau());
   }

   public Point3d getContactPosition()
   {
      if (zSlider == null || xSlider == null)
         return new Point3d();
      return new Point3d(xSlider.getQ(), 0.0, zSlider.getQ());
   }

   public double getTheta1()
   {
      return ankle.getQ();
   }

   public double getTheta1d()
   {
      return ankle.getQD();
   }

   public double getTheta1dd()
   {
      return ankle.getQDD();
   }

   public void setTheta1(double theta1)
   {
      ankle.setQ(theta1);
   }

   public void setTheta1d(double theta1d)
   {
      ankle.setQd(theta1d);
   }

   public void setTheta1dd(double theta1dd)
   {
      ankle.setQdd(theta1dd);
   }

   public double getTheta2()
   {
      return hip.getQ();
   }

   public double getTheta2d()
   {
      return hip.getQD();
   }

   public double getTheta2dd()
   {
      return hip.getQDD();
   }

   public void setTheta2(double theta2)
   {
      hip.setQ(theta2);
   }

   public void setTheta2d(double theta2d)
   {
      hip.setQd(theta2d);
   }

   public void setTheta2dd(double theta2dd)
   {
      hip.setQdd(theta2dd);
   }

   public void setGRFx(double force)
   {
      if (xSlider == null)
         return;
      xSlider.setTau(force);
   }

   public void setGRFz(double force)
   {
      if (zSlider == null)
         return;
      zSlider.setTau(force - DUMMY_MASS * getGravityZ());
   }

}
