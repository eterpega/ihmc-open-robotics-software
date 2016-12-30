package us.ihmc.robotics.dataStructures.parameter;

public interface ParameterVisitor
{
   void visitBoolean(BooleanParameter parameter);
   void visitDoubleArray(DoubleArrayParameter parameter);
   void visitDouble(DoubleParameter parameter);
   void visitIntegerArray(IntegerArrayParameter parameter);
   void visitInteger(IntegerParameter parameter);
   void visitString(StringParameter parameter);
}
