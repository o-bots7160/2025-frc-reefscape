package frc.robot.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.DoubleCircularBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pose2dFilter
{
   class AverageBuffer extends DoubleCircularBuffer
   {
      private int    maxSize;
      private double sum;
      public AverageBuffer( int size )
      {
         super( size );
         maxSize = size;
      }
      @Override
      public void addFirst( double value )
      {
         if ( size() >= maxSize )
         {
            sum -= super.getLast();
         }
         sum += value;
         super.addFirst( value );
      }
      @Override
      public void addLast( double value )
      {
         if ( size() >= maxSize )
         {
            sum -= super.getFirst();
         }
         sum += value;
         super.addLast( value );
      }
      @Override
      public void clear()
      {
         sum = 0.0;
         super.clear();
      }
      @Override
      public double removeFirst( )
      {
         double temp = 0.0;

         temp = super.removeFirst( );
         sum -= temp;

         return temp;
      }
      @Override
      public double removeLast( )
      {
         double temp = 0.0;

         temp = super.removeLast();
         sum -= temp;

         return temp;
      }
      @Override
      public void resize( int new_size )
      {
         maxSize = new_size;
         super.resize( new_size );
      }
      public double average( double value )
      {
         addLast( value );
         return sum / size();
      }
      public double average( )
      {
         return sum / size();
      }
   }
   final private int    maxSamples = 50;
   final private double maxAge     =  10.0; // seconds
   final private double minPeriod  =  2.0; // seconds

   final private long    minSamples = (long)( (double)maxSamples / 4.0 );

   private AverageBuffer buffX  = new AverageBuffer( maxSamples     );
   private AverageBuffer buffY  = new AverageBuffer( maxSamples     );
   private AverageBuffer buffRx = new AverageBuffer( maxSamples     );
   private AverageBuffer buffRy = new AverageBuffer( maxSamples     );
   private AverageBuffer buffT  = new AverageBuffer( maxSamples     );
   private AverageBuffer buffTd = new AverageBuffer( maxSamples - 1 );

   public  Pose2d  avgPose;
   public  double  avgTime;
   public  double  avgPeriod; 
   private boolean verbosity;
   //
   //   Constructor prepare to filter poses collected from vision (or other)
   //   system
   //
   //
   public Pose2dFilter( )
   {
      verbosity = false; 
      reset();
   }
   public Pose2dFilter( boolean new_verbosity)
   {
      verbosity = new_verbosity;  
      reset();
   }
   //
   //   Clear all sums and buffers
   //
   //
   public void reset()
   {
      buffX .clear();
      buffY .clear();
      buffRx.clear();
      buffRy.clear();
      buffT .clear();
      buffTd.clear();
      avgPose    = new Pose2d();
      avgTime    = 0.0;
      avgPeriod  = 0.0;
   }
   //
   //   Add a pose to the averaging history and return true if we have enough
   //   samples for a good average (and they aren't too spread out) and false
   //   otherwise
   //
   //   Results are populated into:
   //      avgPose: average X, Y, and heading of samples
   //      avgTime: average of timestamp to pass to pose estimator
   //
   //   Also calculated for the callers information is:
   //      avgPeriod:  average time between readings
   //
   public boolean addData( Pose2d newSample, double timestamp )
   {
      double  rotation = newSample.getRotation().getRadians();
      //
      // Clean out data older that maxAge seconds??? Is this too long? Too short?
      //
      //
      while ( buffT.size() > 0 && ( timestamp - buffT.getFirst() ) > maxAge )
      {
         buffX .removeFirst();
         buffY .removeFirst();
         buffRx.removeFirst();
         buffRy.removeFirst();
         buffT .removeFirst();
         buffTd.removeFirst();
      }
      avgPose   = new Pose2d( buffX.average( newSample.getX() ),
                              buffY.average( newSample.getY() ),
                              // Clever solution I hadn't seen before. See wikipedia article entitle "Circular Mean"
                              new Rotation2d( Math.atan2( buffRx.average( Math.sin( rotation ) ), buffRy.average( Math.cos( rotation ) ) ) ) );
      avgPeriod = buffTd.average( timestamp - buffT.getLast() );
      avgTime   = buffT.average(  timestamp );

      if ( verbosity )
      {
         SmartDashboard.putNumber( "Pose2dFilter/Samples", buffX.size( )   );
         SmartDashboard.putNumber( "Pose2dFilter/Period",  avgPeriod       );
         SmartDashboard.putNumber( "Pose2dFilter/X",       avgPose.getX( ) );
         SmartDashboard.putNumber( "Pose2dFilter/Y",       avgPose.getY( ) );
         SmartDashboard.putNumber( "Pose2dFilter/Heading", avgPose.getRotation( ).getDegrees( ) );
      }
      
      return ( buffX.size() > minSamples && ( avgPeriod < minPeriod ) );
   }
}