package us.ihmc.robotDataVisualizer.graphics;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.tools.processManagement.JavaProcessSpawner;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class TestVisualizer
{
   private static PlanarRegionsList createPlanaRegionsList()
   {

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      transformToWorld.setTranslation(0.0, 0.0, 0.2);
      java.util.List<ConvexPolygon2D> polygons1 = new ArrayList<>();
      polygons1.add(new ConvexPolygon2D(createCircle(0.1)));
      polygons1.add(new ConvexPolygon2D(createCircle(new Point2D(0.5, -0.2), 0.35)));
      polygons1.add(new ConvexPolygon2D(createCircle(new Point2D(-0.5, -0.1), 0.4)));
      PlanarRegion planarRegion1 = new PlanarRegion(transformToWorld, polygons1);
      transformToWorld.setRotationRollAndZeroTranslation(0.3 * Math.PI);
      transformToWorld.setTranslation(0.0, 0.3, 0.1);
      PlanarRegion planarRegion2 = new PlanarRegion(transformToWorld, new ConvexPolygon2D(createCircle(0.3)));
      Random random = new Random(45L);
      transformToWorld = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      PlanarRegion planarRegion3 = new PlanarRegion(transformToWorld, new ConvexPolygon2D(createCircle(0.3)));
      transformToWorld = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      PlanarRegion planarRegion4 = new PlanarRegion(transformToWorld, new ConvexPolygon2D(createCircle(0.3)));
      transformToWorld = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      PlanarRegion planarRegion5 = new PlanarRegion(transformToWorld, new ConvexPolygon2D(createCircle(0.3)));

      return new PlanarRegionsList(planarRegion1, planarRegion2, planarRegion3, planarRegion4, planarRegion5);
   }

   private static List<Point2D> createCircle(double radius)
   {
      return createCircle(new Point2D(), radius);
   }

   private static List<Point2D> createCircle(Point2D center, double radius)
   {
      List<Point2D> vertices = new ArrayList<>();

      int numberOfPoints = 10;
      for (int i = 0; i < numberOfPoints; i++)
      {
         double angle = i / (double) numberOfPoints * 2.0 * Math.PI;
         double x = center.getX() + radius * Math.cos(angle);
         double y = center.getY() + radius * Math.sin(angle);
         vertices.add(new Point2D(x, y));
      }
      return vertices;
   }

   private static YoGraphicPolygon createYoGraphicPolygon(YoVariableRegistry registry)
   {
      YoFrameConvexPolygon2d convexPolygon2d = new YoFrameConvexPolygon2d("poupou", ReferenceFrame.getWorldFrame(), 30, registry);
      ConvexPolygon2D polygon = new ConvexPolygon2D(createCircle(new Point2D(50.0, 50.0), 100.0));
      convexPolygon2d.setConvexPolygon2d(polygon);
      return new YoGraphicPolygon("poly", convexPolygon2d, "shnoup", "", registry, 1.0, YoAppearance.Red());
   }

   public static void main(String[] args)
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      YoGraphicPolygon polygon = createYoGraphicPolygon(registry);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.registerYoGraphic("polygon", polygon);

      PeriodicThreadSchedulerFactory scheduler = new PeriodicNonRealtimeThreadSchedulerFactory();

      final YoVariableServer yoVariableServer = new YoVariableServer(TestVisualizer.class, scheduler, null, LogSettings.TOOLBOX, 0.001);
      yoVariableServer.setMainRegistry(registry, null, yoGraphicsListRegistry);
      yoVariableServer.start();

      Runnable command = new Runnable()
      {
         long count = 0L;

         @Override public void run()
         {
            yoVariableServer.update(count += Conversions.millisecondsToNanoseconds(10L));
         }
      };

      Executors.newSingleThreadScheduledExecutor().scheduleAtFixedRate(command, 0L, 10L, TimeUnit.MILLISECONDS);

      JavaProcessSpawner spawner = new JavaProcessSpawner(true, true);
      spawner.spawn(YoGraphicServerVisualizer.class);
   }
}
