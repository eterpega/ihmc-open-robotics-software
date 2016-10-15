package us.ihmc.robotbuilder.util;

import javafx.application.Platform;
import javafx.geometry.Bounds;
import javafx.scene.Node;
import javafx.scene.PerspectiveCamera;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Translate;
import javaslang.concurrent.Future;
import javaslang.concurrent.Promise;
import us.ihmc.javaFXToolkit.JavaFXTools;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.util.concurrent.Callable;

/**
 * Utility methods
 */
public class Util
{
   /**
    * Wraps {@link Platform#runLater(Runnable)} to use a {@link Future} which allows chaining
    * and waiting for actions.
    * @param callable code to run in the UI thread
    * @param <T> return type of the UI code
    * @return {@link Future} instance for the given code that is fulfilled as soon as the given code executes in the UI thread
    */
   public static <T> Future<T> runLaterInUI(Callable<? extends T> callable)
   {
      Promise<T> result = Promise.make();
      Platform.runLater(() ->
                        {
                           try
                           {
                              result.success(callable.call());
                           }
                           catch (Throwable thr)
                           {
                              result.failure(thr);
                           }
                        });
      return result.future();
   }

   /**
    * Computes camera settings so that the camera shows the whole node (based on its bounding box) from the direction
    * given by the dir vector if corner == false or from the direction of a bounding box corner if corner == true
    * @param node node to look at
    * @param fovDegrees camera field of view in degrees
    * @param dir direction where to look at the node from
    * @param up up vector
    */
   public static PerspectiveCamera lookAtNodeFromDirection(final Node node, double fovDegrees, final Vector3d dir, final Vector3d up)
   {
      // We get a bounding sphere of the node and calculate how far the camera needs
      // to be to fit in the whole sphere in its view.
      final Bounds box = node.getBoundsInParent();
      if (box.isEmpty())
      {
         return new PerspectiveCamera(true);
      }

      Vector3d dirVec = new Vector3d(dir);
      dirVec.normalize();
      Vector3d bboxMin = new Vector3d(box.getMinX(), box.getMinY(), box.getMinZ());
      Vector3d bboxMax = new Vector3d(box.getMaxX(), box.getMaxY(), box.getMaxZ());
      Vector3d sceneCenter = new Vector3d(bboxMin);
      sceneCenter.add(bboxMax);
      sceneCenter.scale(0.5f);

      Vector3d radiusVec = new Vector3d(sceneCenter);
      radiusVec.sub(bboxMin);
      double r = radiusVec.length(); // scene bounding sphere radius
      double near = r / 1000; // camera near, giving some space for zoom-in
      double h = Math.sin(Math.toRadians(fovDegrees)) * near; // height of the projection plane
      double d = r * near / h; // how far we need to be for the bounding sphere to fit in the projection rectangle

      double far = 10 * (2 * r + d); // (distance from the scene + scene bounding sphere diameter) * 10 (allows a 10x zoom-out)

      Vector3d cameraLocation = new Vector3d(dirVec);
      cameraLocation.scale(-d);
      cameraLocation.add(sceneCenter);

      PerspectiveCamera result = new PerspectiveCamera(true);
      result.setFieldOfView(fovDegrees);
      result.setNearClip(near);
      result.setFarClip(far);

      result.getTransforms().add(new Translate(cameraLocation.x, cameraLocation.y, cameraLocation.z));
      result.getTransforms().add(lookAt(cameraLocation, sceneCenter, up));
      return result;
   }

   /**
    * Get the rotation matrix from a camera location, target location and camera up vector.
    * Returns a 4x4 affine matrix for convenience.
    * @param cameraPosition camera location
    * @param target target location
    * @param up up vector
    * @return transform representing the rotation
    */
   private static Affine lookAt(Vector3d cameraPosition, Vector3d target, Vector3d up)
   {
      Vector3d camDirection = new Vector3d(target);
      camDirection.sub(cameraPosition);
      camDirection.normalize();

      Vector3d zAxis = new Vector3d(camDirection);
      zAxis.normalize();

      Vector3d xAxis = new Vector3d();

      Vector3d down = new Vector3d(up);
      down.negate();

      xAxis.cross(down, zAxis);
      xAxis.normalize();

      Vector3d yAxis = new Vector3d();
      yAxis.cross(zAxis, xAxis);

      Matrix3d rotation = new Matrix3d();
      rotation.setColumn(0, xAxis);
      rotation.setColumn(1, yAxis);
      rotation.setColumn(2, zAxis);
      Affine result = new Affine();
      result.appendTranslation(cameraPosition.x, cameraPosition.y, cameraPosition.z);
      JavaFXTools.convertRotationMatrixToAffine(rotation, result);

      return result;
   }

}
