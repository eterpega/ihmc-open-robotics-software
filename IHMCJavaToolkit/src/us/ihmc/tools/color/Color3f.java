package us.ihmc.tools.color;

import java.awt.Color;
import java.io.Serializable;

public class Color3f implements Serializable
{
   private static final long serialVersionUID = -7598000307366567618L;

   private float x, y, z;

   public Color3f()
   {
   }

   public Color3f(float x, float y, float z)
   {
      set(x, y, z);
   }

   public Color3f(float[] v)
   {
      set(v);
   }

   public Color3f(Color3f v1)
   {
      set(v1);
   }

   public Color3f(Color color)
   {
      set(color);
   }
   
   public void set(Color3f v1)
   {
      x = v1.getX();
      y = v1.getY();
      z = v1.getZ();
   }

   public void set(float x, float y, float z)
   {
      this.x = x;
      this.y = y;
      this.z = z;
   }

   public void set(float[] v)
   {
      this.x = v[0];
      this.y = v[1];
      this.z = v[2];
   }

   public final void set(Color color)
   {
      x = (float) color.getRed() / 255.0f;
      y = (float) color.getGreen() / 255.0f;
      z = (float) color.getBlue() / 255.0f;
   }

   public final Color getAwtColor()
   {
      int r = Math.round(x * 255.0f);
      int g = Math.round(y * 255.0f);
      int b = Math.round(z * 255.0f);

      return new Color(r, g, b);
   }

   public float getX()
   {
      return x;
   }

   public void setX(float x)
   {
      this.x = x;
   }

   public float getY()
   {
      return y;
   }

   public void setY(float y)
   {
      this.y = y;
   }

   public float getZ()
   {
      return z;
   }

   public void setZ(float z)
   {
      this.z = z;
   }
}
