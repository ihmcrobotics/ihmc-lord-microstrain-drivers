package us.ihmc.sensors.imu.lord.microstrain;

import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.logging.Logger;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class MicroStrainUDPPacketListener implements Runnable
{
   private final static Logger log = Logger.getLogger(MicroStrainUDPPacketListener.class.getName());

   private static final byte ADAPTIVE_KALMAN_FILTERED_IMU_PACKET = (byte) 0x82;
   private static final byte CF_MIP_IMU_PACKET = (byte) 0x80;

   private static final byte EKF_ESTIMATED_LINEAR_ACCELERATION_DESCRIPTOR = 0x0D;
   private static final byte EKF_ESTIMATED_ANGULAR_RATE_DESCRIPTOR = 0x0E;
   private static final byte EKF_MATRIX_DESCRIPTOR = 0x04;
   private static final byte EKF_QUATERNION_DESCRIPTOR = 0x03;

   private static final byte CF_ESTIMATED_LINEAR_ACCELERATION_DESCRIPTOR = 0x04;
   private static final byte CF_ESTIMATED_ANGULAR_RATE_DESCRIPTOR = 0x05;
   private static final byte CF_MATRIX_DESCRIPTOR = 0x09;
   private static final byte CF_NORTH_VECTOR_DESCRIPTOR = 0x10;
   private static final byte CF_QUATERNION_DESCRIPTOR = 0x0A;

   private DatagramChannel receiveChannel;
   private volatile boolean requestStop = false;
   private final ByteBuffer receiveBuffer = ByteBuffer.allocate(1024);

   private final ConcurrentCopier<MicroStrainData> adaptiveEKFMicrostrainBuffer = new ConcurrentCopier<>(new MicroStrainDataBuilder());
   private final ConcurrentCopier<MicroStrainData> originalMIPMicrostrainBuffer = new ConcurrentCopier<>(new MicroStrainDataBuilder());

   public MicroStrainUDPPacketListener(int port) throws IOException
   {
      receiveChannel = DatagramChannel.open();
      receiveChannel.socket().setReceiveBufferSize(65535);
      receiveChannel.socket().bind(new InetSocketAddress(port));
   }

   private boolean isChecksumValid(ByteBuffer buffer)
   {
      int intA = 0;
      int intB = 0;

      for (int i = buffer.position(); i < (buffer.limit() - 2); i++)
      {
         intA = (intA + ((int) buffer.get() & 0xFF)) & 0xFF;
         intB = (intB + (intA & 0xFF)) & 0xFF;
      }

      byte A = (byte) (intA & 0xFF);
      byte B = (byte) (intB & 0xFF);

      byte checkA = buffer.get();
      byte checkB = buffer.get();

      return (A == checkA && B == checkB);
   }

   private void readOriginalMIPPacketFields(ByteBuffer buffer)
   {
      MicroStrainData data = originalMIPMicrostrainBuffer.getCopyForWriting();
      data.setFilterType(MicroStrainData.MicrostrainFilterType.COMPLIMENTARY_FILTER);

      long time = RealtimeThread.getCurrentMonotonicClockTime();
      data.setReceiveTime(time);

      while (buffer.position() < buffer.limit() - 2)
      {
         int fieldLength = buffer.get();
         int descriptor = buffer.get();
         switch (descriptor)
         {
         case CF_MATRIX_DESCRIPTOR:
            data.setOrientationMatrix(buffer.getFloat(), buffer.getFloat(), buffer.getFloat(), buffer.getFloat(), buffer.getFloat(), buffer.getFloat(),
                                      buffer.getFloat(), buffer.getFloat(), buffer.getFloat());
            break;
         case CF_QUATERNION_DESCRIPTOR:
            data.setQuaternion(buffer.getFloat(), buffer.getFloat(), buffer.getFloat(), buffer.getFloat());
            break;
         case CF_ESTIMATED_LINEAR_ACCELERATION_DESCRIPTOR:
            data.setLinearAcceleration(buffer.getFloat(), buffer.getFloat(), buffer.getFloat());
            break;
         case CF_ESTIMATED_ANGULAR_RATE_DESCRIPTOR:
            data.setAngularRate(buffer.getFloat(), buffer.getFloat(), buffer.getFloat());
            break;
         case CF_NORTH_VECTOR_DESCRIPTOR:
            data.setGeomagneticNorthVector(buffer.getFloat(), buffer.getFloat(), buffer.getFloat());
         default:
            log.warning("Unknown field " + Integer.toHexString(descriptor));
            buffer.position(buffer.position() + fieldLength);
            break;
         }
      }
      originalMIPMicrostrainBuffer.commit();
   }

   private void readKalmanFilteredPacketFields(ByteBuffer buffer)
   {
      MicroStrainData data = adaptiveEKFMicrostrainBuffer.getCopyForWriting();
      data.setFilterType(MicroStrainData.MicrostrainFilterType.ADAPTIVE_EKF);

      long time = RealtimeThread.getCurrentMonotonicClockTime();
      data.setReceiveTime(time);

      while (buffer.position() < buffer.limit() - 2)
      {
         int fieldLength = buffer.get();
         int descriptor = buffer.get();
         short isValidBytes;
         switch (descriptor)
         {
         case EKF_MATRIX_DESCRIPTOR:
            data.setOrientationMatrix(buffer.getFloat(), buffer.getFloat(), buffer.getFloat(), buffer.getFloat(), buffer.getFloat(), buffer.getFloat(),
                                      buffer.getFloat(), buffer.getFloat(), buffer.getFloat());
            isValidBytes = buffer.getShort();
            data.setMatrixValid(isMeasurementValid(isValidBytes));
            break;
         case EKF_QUATERNION_DESCRIPTOR:
            data.setQuaternion(buffer.getFloat(), buffer.getFloat(), buffer.getFloat(), buffer.getFloat());
            isValidBytes = buffer.getShort();
            data.setQuaternionValid(isMeasurementValid(isValidBytes));
            break;
         case EKF_ESTIMATED_LINEAR_ACCELERATION_DESCRIPTOR:
            data.setLinearAcceleration(buffer.getFloat(), buffer.getFloat(), buffer.getFloat());
            isValidBytes = buffer.getShort();
            data.setAccelerationValid(isMeasurementValid(isValidBytes));
            break;
         case EKF_ESTIMATED_ANGULAR_RATE_DESCRIPTOR:
            data.setAngularRate(buffer.getFloat(), buffer.getFloat(), buffer.getFloat());
            isValidBytes = buffer.getShort();
            data.setAngularRateValid(isMeasurementValid(isValidBytes));
            break;
         default:
            log.warning("Unknown field " + Integer.toHexString(descriptor));
            buffer.position(buffer.position() + fieldLength);
            break;
         }
      }
      adaptiveEKFMicrostrainBuffer.commit();
   }

   private boolean isMeasurementValid(short isValid)
   {
      return isValid == 0x0001;
   }

   public MicroStrainData getLatestData(MicroStrainData.MicrostrainFilterType packetType)
   {
      switch (packetType)
      {
      case ADAPTIVE_EKF:
         return adaptiveEKFMicrostrainBuffer.getCopyForReading();
      case COMPLIMENTARY_FILTER:
         return originalMIPMicrostrainBuffer.getCopyForReading();
      default:
         return null;
      }
   }

   @Override
   public void run()
   {
      while (!requestStop)
      {
         try
         {
            receiveBuffer.clear();
            receiveChannel.receive(receiveBuffer);
            receiveBuffer.flip();
            if (!isChecksumValid(receiveBuffer))
            {
               log.warning("Invalid checksum");
               continue;
            }

            byte descriptor = receiveBuffer.get(2);
            receiveBuffer.position(4);
            switch (descriptor)
            {
            case ADAPTIVE_KALMAN_FILTERED_IMU_PACKET:
               readKalmanFilteredPacketFields(receiveBuffer);
               break;
            case CF_MIP_IMU_PACKET:
               readOriginalMIPPacketFields(receiveBuffer);
               break;
            default:
               log.warning("Unknown packet type  " + (descriptor & 0xff));
            }
         }
         catch (IOException e)
         {
            e.printStackTrace();
            requestStop = true;
         }
      }
   }

   public void stop()
   {
      requestStop = true;
   }

   private static MicroStrainUDPPacketListener create(long serialNumber) throws IOException
   {
      int port = 50000 + (int) (serialNumber % 1000);
      log.info("Connecting to IMU on port " + port);
      return new MicroStrainUDPPacketListener(port);
   }

   public static MicroStrainUDPPacketListener createRealtimeListener(PriorityParameters priority, long serialNumber) throws IOException
   {
      MicroStrainUDPPacketListener listener = create(serialNumber);
      new RealtimeThread(priority, listener).start();
      return listener;
   }

   public static MicroStrainUDPPacketListener createNonRealtimeListener(long serialNumber) throws IOException
   {
      MicroStrainUDPPacketListener listener = create(serialNumber);
      new Thread(listener).start();
      return listener;
   }

   private static class MicroStrainDataBuilder implements Builder<MicroStrainData>
   {

      @Override
      public MicroStrainData newInstance()
      {
         return new MicroStrainData();
      }

   }

   public static void main(String[] args) throws IOException
   {
      new Thread(new MicroStrainUDPPacketListener(50571)).start();
   }
}
