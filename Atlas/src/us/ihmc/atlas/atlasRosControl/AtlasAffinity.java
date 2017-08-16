package us.ihmc.atlas.atlasRosControl;

import java.util.logging.Logger;

import us.ihmc.affinity.CPUTopology;
import us.ihmc.affinity.Package;
import us.ihmc.affinity.Processor;

public class AtlasAffinity
{
   private final static Logger log = Logger.getLogger(AtlasAffinity.class.getName());
   
   private final boolean setAffinity;
  
   private final Processor estimatorThreadProcessor;
   private final Processor controlThreadProcessor;
   
   public AtlasAffinity(boolean setAffinity)
   {
      this.setAffinity = setAffinity;

      if(this.setAffinity)
      {
         CPUTopology topology = new CPUTopology();

         if (topology.isHyperThreadingEnabled())
         {
            log.severe("WARNING: Hyper-Threading is enabled. Expect higher amounts of jitter");
         }

         log.config("Pinning control threads to processor 1 & 2.");
         Package socket = topology.getPackage(0);
         estimatorThreadProcessor = socket.getCore(1).getDefaultProcessor();
         controlThreadProcessor = socket.getCore(2).getDefaultProcessor();
      }
      else
      {
         estimatorThreadProcessor = null;
         controlThreadProcessor = null;
      }
   }

   public Processor getEstimatorThreadProcessor()
   {
      if(!setAffinity)
      {
         throw new RuntimeException("Setting affinity is disabled");
      }
      return estimatorThreadProcessor;
   }

   public Processor getControlThreadProcessor()
   {
      if(!setAffinity)
      {
         throw new RuntimeException("Setting affinity is disabled");
      }
      return controlThreadProcessor;
   }

   public boolean setAffinity()
   {
      return setAffinity;
   }
   
   
}
