package us.ihmc.visualizers.sensors.imu.lord.microstrain;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.sensors.imu.lord.microstrain.MicroStrainData;
import us.ihmc.sensors.imu.lord.microstrain.MicroStrainUDPPacketListener;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.io.IOException;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class MicroStrain3DMVisualizer
{

   public static void main(String[] args) throws IOException
   {
      long serialNumber = Long.parseLong(args[0]);

      final MicroStrain3DMRobot robot = new MicroStrain3DMRobot(serialNumber);

      robot.setController(robot);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setGroundVisible(false);
      scs.addYoGraphic(robot.getMagneticNorthGraphicVector());
      scs.startOnAThread();

   }

   private static class MicroStrain3DMRobot extends Robot implements RobotController
   {
      private static double MS3DM_MASS = 0.0018;
      private static double MS3DM_LENGTH = 0.044;
      private static double MS3DM_WIDTH = 0.025;
      private static double MS3DM_HEIGHT = 0.011;

      private FloatingJoint ms3DM;
      private final RotationMatrix rotation = new RotationMatrix();

      private YoVariableRegistry registry = new YoVariableRegistry("MicroStrain3DMData");

      private YoDouble yaw = new YoDouble("yaw", registry);
      private YoDouble pitch = new YoDouble("pitch", registry);
      private YoDouble roll = new YoDouble("roll", registry);

      private YoDouble wz = new YoDouble("wz", registry);
      private YoDouble wy = new YoDouble("wy", registry);
      private YoDouble wx = new YoDouble("wx", registry);

      private YoDouble xdd = new YoDouble("xdd", registry);
      private YoDouble ydd = new YoDouble("ydd", registry);
      private YoDouble zdd = new YoDouble("zdd", registry);

      private final RotationMatrix temporaryMatrix = new RotationMatrix();
      private final MicroStrainUDPPacketListener listener;
      private final YoFramePoint3D magneticNorthOrigin;
      private final YoFrameVector3D magneticNorthVector;
      private final YoGraphicVector magneticNorth;

      private MicroStrain3DMRobot(long serialNumber) throws IOException
      {
         super("MicroStrain3DMRobot");

         listener = MicroStrainUDPPacketListener.createNonRealtimeListener(serialNumber);

         ms3DM = new FloatingJoint("ms3DM", new Vector3D(0.0, 0.0, 0.0), this);
         ms3DM.setLink(MS3DMLink());

         this.addRootJoint(ms3DM);

         this.setGravity(0.0, 0.0, 0.0);

         magneticNorthOrigin = new YoFramePoint3D("magneticNorthOrigin", ReferenceFrame.getWorldFrame(), registry);
         magneticNorthOrigin.set(0.0, 0.0, 0.0);
         magneticNorthVector = new YoFrameVector3D("magneticNorthVector", ReferenceFrame.getWorldFrame(), registry);
         magneticNorthVector.set(1.0, 1.0, 1.0);
         magneticNorth = new YoGraphicVector("magneticNorthVector", magneticNorthOrigin, magneticNorthVector, 0.5, YoAppearance.Chartreuse(), true);
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return "updater";
      }

      @Override
      public String getDescription()
      {
         return getName();
      }

      @Override
      public void doControl()
      {
         MicroStrainData data = listener.getLatestData(MicroStrainData.MicrostrainFilterType.COMPLIMENTARY_FILTER);
         this.set(data.getLinearAcceleration(), data.getAngularRate(), data.getQuaternion());
         magneticNorthVector.set(data.getGeomagneticNorthVector());
         ThreadTools.sleep(1);
      }

      public YoGraphicVector getMagneticNorthGraphicVector()
      {
         return this.magneticNorth;
      }

      private Link MS3DMLink()
      {
         Link p = new Link("ms3DMLink");
         p.setMass(MS3DM_MASS);
         p.setComOffset(0.0, 0.0, 0.0);
         p.setMomentOfInertia(0.1, 0.1, 0.1);

         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.addCoordinateSystem(0.1);
         linkGraphics.addCube(MS3DM_LENGTH, MS3DM_WIDTH, MS3DM_HEIGHT, YoAppearance.Purple());

         p.setLinkGraphics(linkGraphics);

         return p;

      }

      private void set(Vector3DReadOnly accel, Vector3DReadOnly angRate, QuaternionReadOnly orientation)
      {
         temporaryMatrix.set(orientation);
         rotation.set(MicroStrainData.MICROSTRAIN_TO_ZUP_WORLD);
         rotation.multiply(temporaryMatrix);

         ms3DM.setRotation(rotation);
         yaw.set(rotation.getYaw());
         pitch.set(rotation.getPitch());
         roll.set(rotation.getRoll());

         xdd.set(accel.getX() * MicroStrainData.MICROSTRAIN_GRAVITY);
         ydd.set(accel.getY() * MicroStrainData.MICROSTRAIN_GRAVITY);
         zdd.set(accel.getZ() * MicroStrainData.MICROSTRAIN_GRAVITY);

         wx.set(angRate.getX());
         wy.set(angRate.getY());
         wz.set(angRate.getZ());

      }
   }
}
