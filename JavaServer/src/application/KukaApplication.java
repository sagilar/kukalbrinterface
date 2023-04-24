package application;

import java.sql.Timestamp;
import java.text.DecimalFormat;
import java.util.TimerTask;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.deviceModel.PositionInformation;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.CIRC;
import com.kuka.roboticsAPI.motionModel.CartesianPTP;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.SPL;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.handGuiding;

public class KukaApplication extends RoboticsAPIApplication {
	@Inject
	private LBR lbr;
	//LBRClient tcpclient = new LBRClient();
	LBRServer lbrServer = null;
	Thread t1;
    java.util.Timer timer1 = new java.util.Timer();
    Thread t2;
    java.util.Timer timer2 = new java.util.Timer();
    String receivedMessage = "";
    boolean finalFlag = false;
    boolean commFlag = false;
    ICondition conditionRestriction;
    double jointSpeedRelative=0.25, jointAccelerationRelative=0, jointJerkRelative=0, blendingRelative=0;
    boolean featuresActive = true;
    
    

	final static double radiusOfCircMove=120;
	final static int nullSpaceAngle = 80;

	final static double offsetAxis2And4=Math.toRadians(20);
	final static double offsetAxis4And6=Math.toRadians(-40);
	double[] loopCenterPosition= new double[]{
			0, offsetAxis2And4, 0, offsetAxis2And4 +offsetAxis4And6 -Math.toRadians(90), 0, offsetAxis4And6,Math.toRadians(90)};


	private final static String informationText=
			"This application is intended for floor mounted robots!"+ "\n" +
			"\n" +
			"This application enables remote control over the robot through a TCP socket.\n";

	public void initialize() {
		lbrServer = new LBRServer();
		Thread server = new Thread(lbrServer);
        server.start();
        try {
			server.join();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	public void run() {
		/* TCP socket */
		/* Threads initialization */
		/**** Writing thread ****/
        t1 = new Thread() {
            @Override
            public void run() {

                timer1.scheduleAtFixedRate(new TimerTask() {
                    @Override
                    public void run() {
                    	
                    	JointPosition positionThd = lbr.getCurrentJointPosition(); // current joint position
                    	JointPosition targetPositionThd = lbr.getCommandedJointPosition(); // target joint position
                    	TorqueSensorData measuredData = lbr.getMeasuredTorque(); // measured sensor data
                    	TorqueSensorData externalData = lbr.getExternalTorque(); // externally acting torque
                    	ForceSensorData fsData = lbr.getExternalForceTorque(lbr.getFlange());
                    	Vector force = fsData.getForce();
                    	PositionInformation cartesianPosition = lbr.getPositionInformation(lbr.getFlange()); // current joint position
                    	Frame currentFrame = cartesianPosition.getCurrentCartesianPosition();
                    	double currentXaxis = currentFrame.getX();
                    	double currentYaxis = currentFrame.getY();
                    	double currentZaxis = currentFrame.getZ();
                    	double forceInX = force.getX();
                    	double forceInY = force.getY();
                    	double forceInZ = force.getZ();
                		double[] positionsThd = positionThd.get();
                		double[] targetPositionsThd = targetPositionThd.get();
                		double[] meDataThd = measuredData.getTorqueValues();
                		double[] exDataThd = externalData.getTorqueValues();
                		Timestamp timestampThd = new Timestamp(System.currentTimeMillis());
                		String sep = ",";
                		String msg = String.format("%.3f",(double)(timestampThd.getTime()/1000.0)) + sep;
                		for (int j= 0; j < positionsThd.length;j++)
                		{
                			msg += String.valueOf(positionsThd[j]) + sep;
                		}
                		for (int i= 0; i < targetPositionsThd.length;i++)
                		{
                			msg += String.valueOf(targetPositionsThd[i]) + sep;
                		}
                		for (int k= 0; k < meDataThd.length;k++)
                		{
                			msg += String.valueOf(meDataThd[k]) + sep;
                		}
                		for (int l= 0; l < exDataThd.length;l++)
                		{
                			msg += String.valueOf(exDataThd[l]) + sep;
                		}
                		msg += String.valueOf(forceInX) + sep;
                		msg += String.valueOf(forceInY) + sep;
                		msg += String.valueOf(forceInZ);
                		LBRServer.sendMessage(msg + "\n");

                    }
                }, 100, 100); // 50 Hz -> 20ms
            }

        };
        
        
        /**** Reading thread ****/
        t2 = new Thread() {
            @Override
            public void run() {

                timer2.scheduleAtFixedRate(new TimerTask() {
                    @Override
                    public void run() {
                    	// updating received messages:
                    	receivedMessage = LBRServer.receiveMessage();
                    	
                    	getLogger().info("Compute spline for lemniscate motion");	                    	
                		Frame startFrame = lbr.getCurrentCartesianPosition(lbr.getFlange());
                		Spline lemniscateSpline = createLemniscateSpline(startFrame).setJointJerkRel(0.5).setCartVelocity(250);
                    	getLogger().info("Received message: " + receivedMessage);	
                    	
                    	if (receivedMessage.contains("error_socket"))
                    	{
                    		getLogger().info("Communication error - socket");
                    		finalFlag = true;
                    	}
                    	
            	    	if(receivedMessage.contains("move1"))
            			{
            				getLogger().info("Move to start position of the lemniscate motion");	
            				PTP ptpToLoopCenter = ptp(loopCenterPosition);
            				ptpToLoopCenter.setJointVelocityRel(0.25);
            				lbr.move(ptpToLoopCenter);
            			}
            			
            			if(receivedMessage.contains("move2"))
            			{
            				getLogger().info("Execute lemniscate motion");
            				lemniscateSpline.setJointVelocityRel(0.25);
            				lbr.move(lemniscateSpline);
            			}
            			
            			if(receivedMessage.contains("move3"))
            			{
            				getLogger().info("Move in nullspace -"+nullSpaceAngle+"?");		
            				Frame centerFrameWithChangedE1_1 = createChildFrameAndSetE1Offset(startFrame,Math.toRadians(-nullSpaceAngle));
            				LIN linToCenterFrameWithE1_1 = lin(centerFrameWithChangedE1_1);
            				linToCenterFrameWithE1_1.setJointVelocityRel(0.25);
            				lbr.move(linToCenterFrameWithE1_1);
            			}
            			
            			if(receivedMessage.contains("move4"))
            			{
            				getLogger().info("Move in nullspace "+nullSpaceAngle+"?");
            				Frame centerFrameWithChangedE1_2 = createChildFrameAndSetE1Offset(startFrame,Math.toRadians(nullSpaceAngle));
            				LIN linToCenterFrameWithE1_2 = lin(centerFrameWithChangedE1_2);
            				linToCenterFrameWithE1_2.setJointVelocityRel(0.25);
            				lbr.move(linToCenterFrameWithE1_2);
            			}
            			
            			if(receivedMessage.contains("move5"))
            			{
            				getLogger().info("Move to start position");
            				LIN linToStartFrame = lin(startFrame);
            				linToStartFrame.setJointVelocityRel(0.25);
            				lbr.move(linToStartFrame);
            			}
            			
            			if(receivedMessage.contains("activatejointrelspeed"))
            			{
            				//command should be: activatefeatures(true/false)
            				getLogger().info("(De)Activation of features");
            				String[] params = extractMessageParametersStr(receivedMessage);
            				featuresActive = Boolean.valueOf(params[0]);
            			}
            			
            			if(receivedMessage.contains("setupjointrelspeed"))
            			{
            				//command should be: setupjointrelspeed(vel)
            				getLogger().info("Setup of joint speed");
            				Double[] params = extractMessageParametersArray(receivedMessage);
            				jointSpeedRelative = params[0];
            			}
            			
            			if(receivedMessage.contains("setupjointrelacceleration"))
            			{
            				//command should be: setupjointrelacceleration(acc)
            				getLogger().info("Setup of joint acceleration");
            				Double[] params = extractMessageParametersArray(receivedMessage);
            				jointAccelerationRelative = params[0];
            			}
            			
            			if(receivedMessage.contains("setupjointreljerk"))
            			{
            				//command should be: setupjointreljerk(jerk)
            				getLogger().info("Setup of joint jerk");
            				Double[] params = extractMessageParametersArray(receivedMessage);
            				jointJerkRelative = params[0];
            			}
            			
            			if(receivedMessage.contains("setupblendingrelative"))
            			{
            				//command should be: setupjointreljerk(jerk)
            				getLogger().info("Setup of blending");
            				Double[] params = extractMessageParametersArray(receivedMessage);
            				blendingRelative = params[0];
            			}
            			
            			if(receivedMessage.contains("moveptpcart"))
            			{
            				//command should be: moveptp(x,y,z,a,b,c)
            				getLogger().info("Move ptp command cartesian-based");
            				Double[] params = extractMessageParametersArray(receivedMessage);
            				Frame commandedFrame=new Frame(params[0],params[1],params[2],
            						Math.toRadians(params[3]),Math.toRadians(params[4]),Math.toRadians(params[5]));
            				lbr.move(ptp(commandedFrame).setJointVelocityRel(0.25)); //update if required
            			}
            			
            			if(receivedMessage.contains("moveptprad"))
            			{
            				//command should be: moveptp(x,y,z,a,b,c)
            				getLogger().info("Move ptp command radian-based");
            				Double[] params = extractMessageParametersArray(receivedMessage);
            				double[] commandedFrame=new double[]{Math.toRadians(params[0]),Math.toRadians(params[1]),
            						Math.toRadians(params[2]),Math.toRadians(params[3]),Math.toRadians(params[4]),
            						Math.toRadians(params[5]),Math.toRadians(params[6])};
            				
            				//update if required:            				
            				if(featuresActive){
            					IMotionContainer cMovement = lbr.moveAsync(ptp(commandedFrame).setJointVelocityRel(jointSpeedRelative)
            							.setJointAccelerationRel(jointAccelerationRelative)
            							.setJointJerkRel(jointJerkRelative)
            							.setBlendingRel(blendingRelative)
            							);
            				}else{
            					IMotionContainer cMovement = lbr.moveAsync(ptp(commandedFrame));
            				}
            				
            				
            			}
            			
            			if(receivedMessage.contains("movelin"))
            			{
            				//command should be: movelin(x,y,z,a,b,c)
            				getLogger().info("Move linear command");
            				Double[] params = extractMessageParametersArray(receivedMessage);
            				Frame commandedFrame=new Frame(params[0],params[1],params[2],params[3],params[4],params[5]);
            				LIN linToCommandedFrame = lin(commandedFrame);
            				linToCommandedFrame.setJointVelocityRel(0.25);
            				
							//update if required:   
            				if(featuresActive){
            					lbr.move(linToCommandedFrame.setJointVelocityRel(jointSpeedRelative)
            							.setJointAccelerationRel(jointAccelerationRelative)
            							.setJointJerkRel(jointJerkRelative)
            							.setBlendingRel(blendingRelative)
            							);
            				}else{
            					lbr.move(linToCommandedFrame);
            				}
            				
            			}
            			
            			if(receivedMessage.contains("movelrel"))
            			{
            				//command should be: movelrel(x,y,z,a,b,c)
            				getLogger().info("Move linear relative command");
            				Double[] params = extractMessageParametersArray(receivedMessage);
            				Frame commandedFrame=new Frame(params[0],params[1],params[2],
            						Math.toRadians(params[3]),Math.toRadians(params[4]),Math.toRadians(params[5]));
            				LIN linToCommandedFrame = lin(commandedFrame);
            				linToCommandedFrame.setJointVelocityRel(0.25);
            				
							//update if required:   
            				if(featuresActive){
            					lbr.move(linToCommandedFrame.setJointVelocityRel(jointSpeedRelative)
            							.setJointAccelerationRel(jointAccelerationRelative)
            							.setJointJerkRel(jointJerkRelative)
            							.setBlendingRel(blendingRelative)
            							);
            				}else{
            					lbr.move(linToCommandedFrame);
            				}
            				
            			}
            			
            			if(receivedMessage.contains("movecirc"))
            			{
            				//command should be: movecirc(x,y,z,a,b,c)
            				getLogger().info("Move circular command");
            				Double[] params = extractMessageParametersArray(receivedMessage);
            				Frame commandedFrame=new Frame(params[0],params[1],params[2],
            						Math.toRadians(params[3]),Math.toRadians(params[4]),Math.toRadians(params[5]));
            				CIRC commandedCIRC = circ(startFrame, commandedFrame);
            				commandedCIRC.setJointVelocityRel(0.25);
            				
							//update if required:   
            				if(featuresActive){
            					lbr.move(commandedCIRC.setJointVelocityRel(jointSpeedRelative)
            							.setJointAccelerationRel(jointAccelerationRelative)
            							.setJointJerkRel(jointJerkRelative)
            							.setBlendingRel(blendingRelative)
            							);
            				}else{
            					lbr.move(commandedCIRC);
            				}
            			}
            			
            			if(receivedMessage.contains("movespl"))
            			{
            				//command should be: movespl(x,y,z,a,b,c)
            				getLogger().info("Move spl command");
            				Double[] params = extractMessageParametersArray(receivedMessage);
            				Frame commandedFrame=new Frame(params[0],params[1],params[2],
            						Math.toRadians(params[3]),Math.toRadians(params[4]),Math.toRadians(params[5]));
            				SPL commandedSPL = spl(commandedFrame);
            				commandedSPL.setJointVelocityRel(0.25);
            				
							//update if required:   
            				if(featuresActive){
            					lbr.move(commandedSPL.setJointVelocityRel(jointSpeedRelative)
            							.setJointAccelerationRel(jointAccelerationRelative)
            							.setJointJerkRel(jointJerkRelative)
            							.setBlendingRel(blendingRelative)
            							);
            				}else{
            					lbr.move(commandedSPL);
            				}
            				
            			}
            			
            			if(receivedMessage.contains("move_spline"))
            			{
							// Method to be done.
            				//command should be: move_spline(parameters not yet defined)
            				getLogger().info("Move spline command");
            			}
            			
            			if(receivedMessage.contains("starthandguiding"))
            			{
            				//command should be: starthandguiding
            				getLogger().info("Starting hand guiding");
            				lbr.setESMState("2");
            				lbr.move(handGuiding());    				
            			}
            			
            			if(receivedMessage.contains("stophandguiding"))
            			{
            				//command should be: stophandguiding
            				getLogger().info("Stopping hand guiding");
            				lbr.setESMState("1");
            				lbr.move(handGuiding());    				
            			}
            			
            			
            			if(receivedMessage.contains("stop"))
            			{
            				getLogger().info("Stopping application");
            				finalFlag = true;
            			}            				
                    }
                }, 0, 100); // checking buffer every 100ms
            }

        };
        t1.start();
        t2.start();
        
		getLogger().info("Show modal dialog and wait for user to confirm");
		while (finalFlag == false){
			// Threads running
		}
		try {
			t1.join();
			t2.join();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	

	private Spline createLemniscateSpline(Frame centerFrame) {

		// Create a new frame with the center frame as parent. Set an offset for the x axis to this parent.
		Frame rightFrame=(new Frame(centerFrame)).setX(2*radiusOfCircMove);

		// Create a new frame with the center frame as parent. Set an offset for the x axis to this parent.
		Frame leftFrame= (new Frame(centerFrame)).setX(-2*radiusOfCircMove);	

		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame topLeftFrame= (new Frame(centerFrame)).setX(-radiusOfCircMove).setY(radiusOfCircMove);		

		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame topRightFrame= (new Frame(centerFrame)).setX(+radiusOfCircMove).setY(radiusOfCircMove);		

		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame bottomRightFrame= (new Frame(centerFrame)).setX(+radiusOfCircMove).setY(-radiusOfCircMove);

		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame bottomLeftFrame= (new Frame(centerFrame)).setX(-radiusOfCircMove).setY(-radiusOfCircMove);

		// Create a spline that describes a lemniscate
		Spline spline = new Spline(
				spl(bottomLeftFrame),
				spl(leftFrame),
				spl(topLeftFrame),
				spl(centerFrame),
				spl(bottomRightFrame),
				spl(rightFrame),
				spl(topRightFrame),
				spl(centerFrame));
		return spline;
	}

	
	private Frame createChildFrameAndSetE1Offset( Frame parent, double offset) {

		// Create a new frame
		Frame childFrame = new Frame(parent);

		// Create new redundancy information
		LBRE1Redundancy newRedundancyInformation = new LBRE1Redundancy().setE1(offset);

		// Add new redundancy information to new frame
		childFrame.setRedundancyInformation(lbr, newRedundancyInformation);
		return childFrame;
	}
	
	private String[] extractMessageParametersStr(String messageIn)
	{
		int initChar = messageIn.indexOf("(");
		String subString = messageIn.substring(initChar);
		subString = subString.replace(")", "");
		subString = subString.replace("\"", "");
		String[] arrayStr = subString.split(",");
		return arrayStr;
	}
	
	private Double[] extractMessageParametersArray(String messageIn)
	{
		int initChar = messageIn.indexOf("(");
		String subString = messageIn.substring(initChar);
		subString = subString.replace("(", "");
		subString = subString.replace(")", "");
		String[] arrayStr = subString.split(",");
		Double[] arrayParams = new Double[arrayStr.length];
		for(int i=0;i<arrayStr.length;i++)
		{
			arrayParams[i] = Double.parseDouble(arrayStr[i]);
		}
		return arrayParams;
	}
	
	private double extractMessageParametersIdx(String messageIn,int index)
	{
		int initChar = messageIn.indexOf("(");
		String subString = messageIn.substring(initChar);
		subString = subString.replace("(", "");
		subString = subString.replace(")", "");
		String[] arrayStr = subString.split(",");
		Double[] arrayParams = new Double[arrayStr.length];
		for(int i=0;i<arrayStr.length;i++)
		{
			arrayParams[i] = Double.parseDouble(arrayStr[i]);
		}
		return arrayParams[index];
	}

}
