package us.ihmc.robotics.immutableRobotDescription;

import org.ejml.data.DenseMatrix64F;
import org.immutables.value.Value.Default;
import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Lazy;
import org.immutables.value.Value.Modifiable;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.geometry.InertiaTools;
import us.ihmc.robotics.immutableRobotDescription.graphics.*;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;

import static us.ihmc.robotics.immutableRobotDescription.graphics.TransformDescription.fromTranslation;

@Immutable @Modifiable public abstract class LinkDescription implements NamedObject, ModifiableObject
{
   @Default public GraphicsGroupDescription getLinkGraphics()
   {
      return GraphicsGroupDescription.empty();
   }

   @Default public double getMass()
   {
      return 0;
   }

   @Default public Vector3d getCenterOfMassOffset()
   {
      return new Vector3d();
   }

   @Default public DenseMatrix64F getMomentOfInertia()
   {
      return new DenseMatrix64F(3, 3); // TODO: this creates a zero matrix, wouldn't a unit matrix be more appropriate?
   }

   @Override public ModifiableLinkDescription toModifiable()
   {
      return ModifiableLinkDescription.create().from(this);
   }

   @Lazy public PrincipalMomentsOfInertia getPrincipalMomentsOfInertia()
   {
      Vector3d principalMomentsOfInertia = new Vector3d();
      Matrix3d principalAxesRotation = new Matrix3d();
      InertiaTools.computePrincipalMomentsOfInertia(getMomentOfInertia(), principalAxesRotation, principalMomentsOfInertia);
      return new PrincipalMomentsOfInertia(principalAxesRotation, principalMomentsOfInertia);
   }

   public static class PrincipalMomentsOfInertia
   {
      private final Matrix3d principalAxesRotation;
      private final Vector3d principalMomentsOfIntertia;

      PrincipalMomentsOfInertia(Matrix3d principalAxesRotation, Vector3d principalMomentsOfIntertia)
      {
         this.principalAxesRotation = principalAxesRotation;
         this.principalMomentsOfIntertia = principalMomentsOfIntertia;
      }

      public Matrix3d getPrincipalAxesRotation()
      {
         return principalAxesRotation;
      }

      public Vector3d getPrincipalMomentsOfIntertia()
      {
         return principalMomentsOfIntertia;
      }
   }

   // TODO: the following methods really do not belong here, move them elsewhere

   public static DenseMatrix64F convertMomentOfInertia(Matrix3d momentOfInertia)
   {
      DenseMatrix64F result = new DenseMatrix64F(3, 3);
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            result.set(i, j, momentOfInertia.getElement(i, j));
         }
      }
      return result;
   }

   // ////////// Graphics from Mass Properties Here ///////////////////////

   /**
    * Adds an ellipsoid representing the mass and inertia of the link at its center of mass.
    * This ellipsoid has a default matte black appearance.
    */
   public void addEllipsoidFromMassProperties(LinkGraphicsDescription linkGraphics)
   {
      //addEllipsoidFromMassProperties(linkGraphics, null);
   }

   /**
    * Adds a coordinate system representation at the center of mass of this link.  The axis of this system
    * have the given length.
    *
    * @param length length in meters of each arm/axis on the coordinate system.
    */
   public GraphicsGroupDescription createCoordinateSystemInCOM(double length)
   {
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.identity();

      Vector3d comOffset = getCenterOfMassOffset();

      linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
      linkGraphics.addCoordinateSystem(length);

      return GraphicsGroupDescription.fromGraphics3DObject(linkGraphics);
   }

   // TODO: what is this? Remove? Not used anywhere...
   public void addEllipsoidFromMassProperties2(LinkGraphicsDescription linkGraphics, AppearanceDefinition appearance)
   {
      PrincipalMomentsOfInertia principalMomentsOfInertia = getPrincipalMomentsOfInertia();

      Vector3d inertiaEllipsoidRadii = InertiaTools.getInertiaEllipsoidRadii(principalMomentsOfInertia.getPrincipalMomentsOfIntertia(), getMass());

      ArrayList<Vector3d> inertiaEllipsoidAxes = new ArrayList<>();

      Matrix3d principalAxesRotation = principalMomentsOfInertia.getPrincipalAxesRotation();

      Vector3d e1 = new Vector3d();
      principalAxesRotation.getColumn(0, e1);
      e1.normalize();
      e1.scale(inertiaEllipsoidRadii.getX());
      inertiaEllipsoidAxes.add(e1);
      Vector3d e2 = new Vector3d();
      principalAxesRotation.getColumn(1, e2);
      e2.normalize();
      e2.scale(inertiaEllipsoidRadii.getY());
      inertiaEllipsoidAxes.add(e2);
      Vector3d e3 = new Vector3d();
      principalAxesRotation.getColumn(2, e3);
      e3.normalize();
      e3.scale(inertiaEllipsoidRadii.getZ());
      inertiaEllipsoidAxes.add(e3);
      Vector3d e4 = new Vector3d(e1);
      e4.negate();
      inertiaEllipsoidAxes.add(e4);
      Vector3d e5 = new Vector3d(e2);
      e5.negate();
      inertiaEllipsoidAxes.add(e5);
      Vector3d e6 = new Vector3d(e3);
      e6.negate();
      inertiaEllipsoidAxes.add(e6);

      double vertexSize = 0.01 * inertiaEllipsoidRadii.length();

      Vector3d comOffset = getCenterOfMassOffset();

      for (Vector3d vector : inertiaEllipsoidAxes)
      {
         linkGraphics.identity();
         linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
         linkGraphics.translate(vector);
         linkGraphics.addCube(vertexSize, vertexSize, vertexSize, appearance);
      }

      ArrayList<Point3d> inertiaOctahedronVertices = new ArrayList<>();

      Point3d p1 = new Point3d(e1);
      inertiaOctahedronVertices.add(p1);
      Point3d p2 = new Point3d(e2);
      inertiaOctahedronVertices.add(p2);
      Point3d p3 = new Point3d(e4);
      inertiaOctahedronVertices.add(p3);
      Point3d p4 = new Point3d(e5);
      inertiaOctahedronVertices.add(p4);
      Point3d p5 = new Point3d(e3);
      inertiaOctahedronVertices.add(p5);
      Point3d p6 = new Point3d(e6);
      inertiaOctahedronVertices.add(p6);

      ArrayList<Point3d> face1 = new ArrayList<>();
      face1.add(p1);
      face1.add(p5);
      face1.add(p4);
      ArrayList<Point3d> face2 = new ArrayList<>();
      face2.add(p4);
      face2.add(p5);
      face2.add(p3);
      ArrayList<Point3d> face3 = new ArrayList<>();
      face3.add(p3);
      face3.add(p5);
      face3.add(p2);
      ArrayList<Point3d> face4 = new ArrayList<>();
      face4.add(p2);
      face4.add(p5);
      face4.add(p1);

      ArrayList<Point3d> face5 = new ArrayList<>();
      face5.add(p4);
      face5.add(p6);
      face5.add(p1);
      ArrayList<Point3d> face6 = new ArrayList<>();
      face6.add(p3);
      face6.add(p6);
      face6.add(p4);
      ArrayList<Point3d> face7 = new ArrayList<>();
      face7.add(p2);
      face7.add(p6);
      face7.add(p3);
      ArrayList<Point3d> face8 = new ArrayList<>();
      face8.add(p1);
      face8.add(p6);
      face8.add(p2);

      linkGraphics.identity();
      linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
      linkGraphics.addPolygon(face1, appearance);
      linkGraphics.addPolygon(face2, appearance);
      linkGraphics.addPolygon(face3, appearance);
      linkGraphics.addPolygon(face4, appearance);
      linkGraphics.addPolygon(face5, appearance);
      linkGraphics.addPolygon(face6, appearance);
      linkGraphics.addPolygon(face7, appearance);
      linkGraphics.addPolygon(face8, appearance);

      //    linkGraphics.identity();
      //    linkGraphics.translate(comOffset.x, comOffset.y, comOffset.z);
      //    linkGraphics.rotate(principalAxesRotation);
      //    linkGraphics.addEllipsoid(inertiaEllipsoidRadii.x, inertiaEllipsoidRadii.y, inertiaEllipsoidRadii.z, appearance);
      //    linkGraphics.identity();

   }

   /**
    * Adds an ellipsoid representing the mass and inertia of the link at its center of mass
    * with the specified appearance.
    *
    * @param appearance Appearance to be used with the ellipsoid.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public GeometryDescription addEllipsoidFromMassProperties(@Nullable MaterialDescription appearance)
   {
      PrincipalMomentsOfInertia principalMomentsOfInertia = getPrincipalMomentsOfInertia();

      Vector3d inertiaEllipsoidRadii = InertiaTools.getInertiaEllipsoidRadii(principalMomentsOfInertia.getPrincipalMomentsOfIntertia(), getMass());

      if (appearance == null)
         appearance = MaterialDescription.black();

      Matrix3d principalAxesRotation = principalMomentsOfInertia.getPrincipalAxesRotation();

      Vector3d comOffset = getCenterOfMassOffset();

      return EllipsoidDescription.builder()
            .transform(TransformDescription.fromRotation(principalAxesRotation).compose(fromTranslation(comOffset)))
            .radiusX(inertiaEllipsoidRadii.getX())
            .radiusY(inertiaEllipsoidRadii.getY())
            .radiusZ(inertiaEllipsoidRadii.getZ())
            .material(appearance)
            .build();

   }

   /**
    * Adds an box representing the mass and inertia of the link at its center of mass
    * with the specified appearance.
    *
    * Specifically, mimics the code from Gazebo to debug SDF loader
    *
    * See https://bitbucket.org/osrf/gazebo/src/0709b57a8a3a8abce3c67e992e5c6a5c24c8d84a/gazebo/rendering/COMVisual.cc?at=default
    *
    * @param appearance Appearance to be used with the ellipsoid.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public GeometryDescription createBoxFromMassProperties(MaterialDescription appearance)
   {
      Vector3d comOffset = getCenterOfMassOffset();

      if (getMass() <= 0 || getMomentOfInertia().get(0, 0) <= 0 || getMomentOfInertia().get(1, 1) <= 0 || getMomentOfInertia().get(2, 2) <= 0
            || getMomentOfInertia().get(0, 0) + getMomentOfInertia().get(1, 1) <= getMomentOfInertia().get(2, 2)
            || getMomentOfInertia().get(1, 1) + getMomentOfInertia().get(2, 2) <= getMomentOfInertia().get(0, 0)
            || getMomentOfInertia().get(0, 0) + getMomentOfInertia().get(2, 2) <= getMomentOfInertia().get(1, 1))
      {
         System.err.println(getName() + " has unrealistic inertia");
         return GeometryDescription.EMPTY;
      }
      else
      {
         DenseMatrix64F momentOfInertia = getMomentOfInertia();
         double mass = getMass();
         double lx = Math.sqrt(6.0 * (momentOfInertia.get(2, 2) + momentOfInertia.get(1, 1) - momentOfInertia.get(0, 0)) / mass);
         double ly = Math.sqrt(6.0 * (momentOfInertia.get(2, 2) + momentOfInertia.get(0, 0) - momentOfInertia.get(1, 1)) / mass);
         double lz = Math.sqrt(6.0 * (momentOfInertia.get(0, 0) + momentOfInertia.get(1, 1) - momentOfInertia.get(2, 2)) / mass);

         TransformDescription transform = fromTranslation(comOffset).compose(fromTranslation(new Vector3d(0, 0, -lz / 2.0)));
         return BoxDescription.builder()
               .transform(transform)
               .material(appearance)
               .width(lx)
               .height(ly)
               .depth(lz)
               .build();
      }
   }

   public static ImmutableLinkDescription.Builder builder()
   {
      return ImmutableLinkDescription.builder();
   }

   public static LinkDescription empty(String name)
   {
      return builder().name(name).build();
   }
}
