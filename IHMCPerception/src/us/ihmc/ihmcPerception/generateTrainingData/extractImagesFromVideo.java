package us.ihmc.ihmcPerception.generateTrainingData;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import org.jcodec.api.JCodecException;
import org.jcodec.api.awt.FrameGrab;
import org.jcodec.common.DemuxerTrack;
import org.jcodec.common.FileChannelWrapper;
import org.jcodec.common.NIOUtils;
import org.jcodec.containers.mp4.demuxer.MP4Demuxer;

public class extractImagesFromVideo
{

   public static void main(String[] args) throws IOException, JCodecException
   {
      new extractImagesFromVideo().st();
   }

   void st() throws IOException, JCodecException
   {
      File file = new File("src/us/ihmc/ihmcPerception/generateTrainingData/SampleVideo.mp4");
      File directory = new File("src/us/ihmc/ihmcPerception/generateTrainingData/extractedTestImages");

      if (!directory.exists())
         directory.mkdir();

      this.getFrame(file);
   }

   void getFrame(File file) throws IOException, JCodecException
   {
      FileChannelWrapper ch = null;
      try
      {
         ch = NIOUtils.readableFileChannel(file);
         FrameGrab frameGrab = new FrameGrab(ch);

         MP4Demuxer demuxer = new MP4Demuxer(ch);
         DemuxerTrack video_track = demuxer.getVideoTrack();

         double duration = video_track.getMeta().getTotalDuration();
         int numberOfFrames = video_track.getMeta().getTotalFrames();
         System.out.println("Video duration: " + duration + " seconds");
         System.out.println("Number of frames: " + numberOfFrames);

         BufferedImage[] frame = new BufferedImage[numberOfFrames];

         System.out.println("Extracting images please wait!");
         for (int i = 0; i < numberOfFrames; i++)
         {
            frame[i] = ((FrameGrab) frameGrab.seekToFramePrecise(i)).getFrame();
            ImageIO.write(frame[i], "jpg", new File("src/us/ihmc/ihmcPerception/generateTrainingData/extractedTestImages/test" + i + ".jpg"));
            if (i % 30 == 0)
               System.out.print(i + " frames extracted\r");
         }

      } finally
      {
         NIOUtils.closeQuietly(ch);
         System.out.println("Done!");
      }
   }
}
