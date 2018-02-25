package us.ihmc.communication.packets;

public class TextToSpeechPacket extends Packet<TextToSpeechPacket>
{
   public static final String FOOTSTEP_COMPLETED = "Finished Taking Footstep";
   public static final String FINISHED_WALKING = "Finished walking";
   public static final String WALKING_ABORTED = "walking aborted";
   public static final String STARTING_CONTROLLER = "starting controller";
   public static final String WALKING = "walking";
   public static final String MOVING_LEFT_ARM = "moving the left arm";
   public static final String MOVING_RIGHT_ARM = "moving the right arm";
   public static final String NETWORKPROCESSOR_ONLINE = "Reestablished Connection To The Network Processor";

   public boolean speakPacket = false;
   public boolean beep = true;

   public StringBuilder textToSpeak = new StringBuilder();

   public TextToSpeechPacket()
   {
   }

   public void setBeep(boolean beep)
   {
      this.beep = beep;
   }

   public void setTextToSpeak(String textToSpeak)
   {
      this.textToSpeak.setLength(0);
      this.textToSpeak.append(textToSpeak);
   }

   public String getTextToSpeakAsString()
   {
      return textToSpeak.toString();
   }

   @Override
   public boolean epsilonEquals(TextToSpeechPacket other, double epsilon)
   {
      return textToSpeak.equals(other.textToSpeak);
   }

   @Override
   public void set(TextToSpeechPacket other)
   {
      textToSpeak.setLength(0);
      textToSpeak.append(other.textToSpeak);
      setPacketInformation(other);
   }
}