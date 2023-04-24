package application;

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.net.Socket;
import java.util.logging.Level;
import java.util.logging.Logger;

public class LBRClient {
	private PrintStream out = null;
	private BufferedReader in = null;
	private Socket socket = null;
	
	
	public void initialize() {
	try {
		socket = new Socket("192.168.1.14", 30001);
	
		in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
		out = new PrintStream(new BufferedOutputStream(socket.getOutputStream()), true);
		} catch (IOException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
		try{
			socket = new Socket("192.168.1.15", 30001);
			
			in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
			out = new PrintStream(new BufferedOutputStream(socket.getOutputStream()), true);
		}catch (IOException ex){
			ex.printStackTrace();
		}
	}
	}
	

	public void close(){
		try {
			out.close();
			in.close();
			socket.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public boolean sendMessage(Object msg)
	{
		try{
			out.println(String.valueOf(msg));
			return true;
		}catch(Exception e){
			return false;
		}
		
	}
	
        public boolean sendMessage(Object msg,int delay)
	{
            try {
                wait(delay);
                out.println(String.valueOf(msg));
                return true;
            } catch (InterruptedException ex) {
                Logger.getLogger(LBRClient.class.getName()).log(Level.SEVERE, null, ex);
                return false;
            }
	}
        
	public String receiveMessage()
	{
            System.out.print("server message: " + in);
            try {
				return in.readLine();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				return "error_socket";
			}
	}
        
        public String receiveMessage(int delay)
	{
            try {
                wait(delay);
                System.out.print("server message: " + in);
                return in.readLine();
            } catch (InterruptedException ex) {
                Logger.getLogger(LBRClient.class.getName()).log(Level.SEVERE, null, ex);
                return "error_socket";
            } catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				return "error_socket";
			}
	}
}

