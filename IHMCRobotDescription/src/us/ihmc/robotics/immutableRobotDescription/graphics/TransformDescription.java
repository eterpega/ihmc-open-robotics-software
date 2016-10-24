package us.ihmc.robotics.immutableRobotDescription.graphics;

import net.jcip.annotations.Immutable;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.*;

/**
 * Represents a 3D transform
 */
@Immutable public class TransformDescription
{
   public static final TransformDescription IDENTITY = new TransformDescription();

   private final @NotNull Matrix4f transform;

   /**
    * Creates a new identity transform
    */
   public TransformDescription()
   {
      transform = new Matrix4f();
      transform.setIdentity();
   }

   public TransformDescription(@Nullable Matrix4f transform)
   {
      if (transform != null)
      {
         this.transform = new Matrix4f(transform); // TODO: get rid of defensive copy when matrix is immutable
      }
      else
      {
         this.transform = new Matrix4f();
         this.transform.setIdentity();
      }
   }

   public TransformDescription(@Nullable RigidBodyTransform transform)
   {
      this.transform = new Matrix4f();
      if (transform != null)
      {
         transform.get(this.transform);
      }
      else
      {
         this.transform.setIdentity();
      }
   }

   /**
    * Create new translation transform
    * @param translation translation vector
    */
   public static TransformDescription fromTranslation(@NotNull Vector3f translation)
   {
      TransformDescription res = new TransformDescription();
      res.transform.setIdentity();
      res.transform.setTranslation(translation);
      return res;
   }

   /**
    * Create new translation transform
    * @param translation translation vector
    */
   public static TransformDescription fromTranslation(@NotNull Vector3d translation)
   {
      return fromTranslation(new Vector3f((float) translation.x, (float) translation.y, (float) translation.z));
   }

   /**
    * Create a new scaling transform
    * @param scale scale
    */
   public static TransformDescription fromScale(float scale)
   {
      TransformDescription res = new TransformDescription();
      res.transform.setIdentity();
      res.transform.setScale(scale);
      return res;
   }

   /**
    * Create a new scaling transform
    * @param scale scale
    */
   public static TransformDescription fromScale(@NotNull Vector3f scale)
   {
      TransformDescription res = new TransformDescription();
      res.transform.setIdentity();
      res.transform.m00 = scale.x;
      res.transform.m11 = scale.y;
      res.transform.m22 = scale.z;
      return res;
   }

   /**
    * Create a new scaling transform
    * @param scale scale
    */
   public static TransformDescription fromScale(@NotNull Vector3d scale)
   {
      return fromScale(new Vector3f((float) scale.x, (float) scale.y, (float) scale.z));
   }

   /**
    * Create a new rotation transform
    * @param angleRad rotation in radians
    * @param axis rotation axis
    */
   public static TransformDescription fromRotation(float angleRad, @NotNull Vector3f axis)
   {
      TransformDescription res = new TransformDescription();
      res.transform.setIdentity();
      if (axis.lengthSquared() > 1e-5)
         res.transform.setRotation(new AxisAngle4f(axis, angleRad));
      return res;
   }

   /**
    * Create a new rotation transform
    * @param yaw x rotation
    * @param pitch y rotation
    * @param roll z rotation
    */
   public static TransformDescription fromRotation(float yaw, float pitch, float roll)
   {
      TransformDescription res = new TransformDescription();
      Matrix4f rotX = new Matrix4f();
      Matrix4f rotY = new Matrix4f();
      Matrix4f rotZ = new Matrix4f();
      rotX.rotX(yaw);
      rotY.rotY(pitch);
      rotZ.rotZ(roll);
      res.transform.setIdentity();
      res.transform.mul(rotX);
      res.transform.mul(rotY);
      res.transform.mul(rotZ);
      return res;
   }

   /**
    * Create a new rotation transform from a rotation matrix
    * @param rotation rotation matrix
    */
   public static TransformDescription fromRotation(Matrix3d rotation)
   {
      TransformDescription res = new TransformDescription();
      res.transform.set(rotation);
      return res;
   }

   /**
    * Create a transform from three base vectors
    * @param x base vector
    * @param y base vector
    * @param z base vector
    * @return transform
    */
   public static TransformDescription fromBaseVectors(Vector3f x, Vector3f y, Vector3f z)
   {
      TransformDescription res = new TransformDescription();
      res.transform.setIdentity();
      res.transform.m00 = x.x;
      res.transform.m01 = x.y;
      res.transform.m02 = x.z;
      res.transform.m10 = y.x;
      res.transform.m11 = y.y;
      res.transform.m12 = y.z;
      res.transform.m20 = z.x;
      res.transform.m21 = z.y;
      res.transform.m22 = z.z;
      return res;
   }

   @NotNull public Matrix4f getMatrix()
   {
      return new Matrix4f(transform); // TODO: remove defensive copy when matrix is immutable
   }

   @NotNull public Vector3f getTranslation()
   {
      return new Vector3f(transform.m30, transform.m31, transform.m32);
   }

   @NotNull public Vector3f getBaseVec1()
   {
      return new Vector3f(transform.m00, transform.m01, transform.m02);
   }

   @NotNull public Vector3f getBaseVec2()
   {
      return new Vector3f(transform.m10, transform.m11, transform.m12);
   }

   @NotNull public Vector3f getBaseVec3()
   {
      return new Vector3f(transform.m20, transform.m21, transform.m22);
   }

   /**
    * Replace the translation vector of this transform with the given one and return the result
    * as a new instance (this instance is not modified).
    * @param translation new translation vector
    * @return new transform with the new translation vector applied
    */
   public TransformDescription withTranslation(Vector3f translation)
   {
      TransformDescription res = new TransformDescription(new Matrix4f(this.transform));
      res.transform.m30 = translation.x;
      res.transform.m31 = translation.y;
      res.transform.m32 = translation.z;
      return res;
   }

   /**
    * Replace the scale of this transform with the given scale and return the result as a new instance.
    * @param scale scale for each axis
    * @return new transform with the scale applied
    */
   public TransformDescription withScale(Vector3f scale)
   {
      Vector3f base1 = getBaseVec1(), base2 = getBaseVec2(), base3 = getBaseVec3();
      base1.normalize();
      base2.normalize();
      base3.normalize();
      TransformDescription res = TransformDescription.fromBaseVectors(base1, base2, base3);
      res.transform.m30 = transform.m30;
      res.transform.m31 = transform.m31;
      res.transform.m32 = transform.m32;
      res.transform.m33 = transform.m33;

      res.transform.m00 *= scale.x;
      res.transform.m01 *= scale.y;
      res.transform.m02 *= scale.z;
      res.transform.m10 *= scale.x;
      res.transform.m11 *= scale.y;
      res.transform.m12 *= scale.z;
      res.transform.m20 *= scale.x;
      res.transform.m21 *= scale.y;
      res.transform.m22 *= scale.z;

      return res;
   }

   /**
    * Replace the scale of this transform with the given scale and return the result as a new instance.
    * @param scale scale
    * @return new transform with the scale applied
    */
   public TransformDescription withScale(float scale)
   {
      return withScale(new Vector3f(scale, scale, scale));
   }

   /**
    * Returns scale factors for the x, y and z axes.
    * @return scale
    */
   public Vector3f getScale()
   {
      return new Vector3f(getBaseVec1().length(), getBaseVec2().length(), getBaseVec3().length());
   }

   /**
    * Returns a new transform with the rotation part replaced by the rotation
    * computed from the three Euler angles (given in radians).
    * @param rotationEulerAngles rotation angles in radians
    * @return transform with replaced rotation
    */
   public TransformDescription withRotation(Vector3f rotationEulerAngles)
   {
      TransformDescription result = TransformDescription.fromRotation(rotationEulerAngles.x, rotationEulerAngles.y, rotationEulerAngles.z);
      result.transform.setTranslation(getTranslation());
      return result.withScale(getScale());
   }

   /**
    * Compose two transforms. The resulting transform is equivalent to first applying
    * this transform and then the given transform.
    * @param transform transform to compose this transform with
    * @return composite transform
    */
   public TransformDescription compose(@NotNull TransformDescription transform)
   {
      if (this == IDENTITY)
         return transform;
      if (transform == IDENTITY)
         return this;
      TransformDescription res = new TransformDescription();
      res.transform.mul(this.transform, transform.transform);
      return res;
   }

   /**
    * Creates a new transform that is inverse of this one.
    * @return inverse transform
    */
   public TransformDescription inverse()
   {
      if (this == IDENTITY)
         return this;
      TransformDescription res = new TransformDescription(new Matrix4f(transform));
      res.transform.invert();
      return res;
   }

   /**
    * Transform a 3D point
    * @param point point to transform
    * @return transformed point
    */
   public Vector3f transformPoint(@NotNull Vector3f point)
   {
      Vector4f pt = new Vector4f(point.x, point.y, point.z, 1);
      transform.transform(pt);
      return new Vector3f(pt.x / pt.w, pt.y / pt.w, pt.z / pt.w);
   }

   /**
    * Get rotation in Euler angles
    * @return yaw, pitch, roll
    */
   public Vector3f getRotationEulerAngles()
   {
      double yaw, pitch = 0, roll = 0;
      if (transform.m00 == 1.0f || transform.m00 == -1.0f)
      {
         yaw = Math.atan2(transform.m02, transform.m23);
      }
      else
      {
         yaw = Math.atan2(-transform.m20, transform.m00);
         pitch = Math.asin(transform.m10);
         roll = Math.atan2(-transform.m12, transform.m11);
      }
      return new Vector3f((float) yaw, (float) pitch, (float) roll);
   }

   /**
    * Transposes this transform
    * @return transposed transform
    */
   public TransformDescription transposed()
   {
      if (this == IDENTITY)
         return this;
      TransformDescription res = new TransformDescription();
      transform.transpose(res.transform);
      return res;
   }

   /**
    * Converts this transform to a double array representing the transformation matrix
    * in row major order.
    * @return matrix
    */
   public double[] toDoubleArrayRowMajor()
   {
      double[] result = new double[16];
      for (int i = 0; i < 4; i++)
      {
         for (int j = 0; j < 4; j++)
         {
            result[4 * i + j] = transform.getElement(i, j);
         }
      }
      return result;
   }
}
