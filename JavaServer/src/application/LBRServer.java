package application;

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;

public class LBRServer implements Runnable {
	private PrintStream out = null;
	private BufferedReader in = null;
	private Socket socket = null;
	private static ServerSocketChannel socketChannel = null;
	private ServerSocketChannel serverChannel;
	private ServerSocket serverSocket = null;
	private static Selector selector = null;
	private static SocketChannel client = null;
	private static ByteBuffer buffer = null;
	private static SelectionKey keyField = null;
	private ServerSocket server = null;
	private DataInputStream inDS = null;
	private String line = "";
	private Map<SocketChannel, byte[]> dataTracking = new HashMap<SocketChannel, byte[]>();
	private static SocketChannel currentClientChannel;
	private static String rcvMsg = "";

	public LBRServer()
    {
        init();
    }

    private void init()
    {
        System.out.println("initializing TCP server");
        if (selector != null) return;
        if (serverChannel != null) return;

        try {
            serverChannel = ServerSocketChannel.open();
            serverChannel.configureBlocking(false);
            serverChannel.socket().bind(new InetSocketAddress("0.0.0.0", 30001));

            selector = Selector.open();

            serverChannel.register(selector, SelectionKey.OP_ACCEPT);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void run()
    {
        System.out.println("Now accepting connections...");
        try {
            while (!Thread.currentThread().isInterrupted()) {

                selector.select(10000);

                Iterator<SelectionKey> keys = selector.selectedKeys().iterator();

                while (keys.hasNext()) {
                    SelectionKey key = keys.next();
                    keys.remove();

                    if (!key.isValid()) {
                        continue;
                    }

                    if (key.isAcceptable()) {
                        System.out.println("Accepting connection");
                        accept(key);
                    }

                    if (key.isWritable()) {
                        write(key);
                    }

                    if (key.isReadable()) {
                        read(key);
                    }
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            closeConnection();
        }
    }

    private void write(SelectionKey key) throws IOException
    {
    	currentClientChannel = (SocketChannel) key.channel();
        byte[] data = dataTracking.get(currentClientChannel);
        dataTracking.remove(currentClientChannel);

        currentClientChannel.write(ByteBuffer.wrap(data));

        key.interestOps(SelectionKey.OP_READ);
    }

    private void closeConnection()
    {
        System.out.println("Closing server down");
        if (selector != null) {
            try {
                selector.close();
                serverChannel.socket().close();
                serverChannel.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    private void accept(SelectionKey key) throws IOException
    {
        ServerSocketChannel serverSocketChannel = (ServerSocketChannel) key.channel();
        SocketChannel socketChannel = serverSocketChannel.accept();
        socketChannel.configureBlocking(false);

        socketChannel.register(selector, SelectionKey.OP_WRITE);
        byte[] hello = "Connection accepted".getBytes();
        dataTracking.put(socketChannel, hello);
    }


    public String read(SelectionKey key) throws IOException
    {
    	currentClientChannel = (SocketChannel) key.channel();
        ByteBuffer readBuffer = ByteBuffer.allocate(1024);
        readBuffer.clear();

        int read;
        try {
            read = currentClientChannel.read(readBuffer);
        } catch (IOException e) {
            e.printStackTrace();
            key.cancel();
            currentClientChannel.close();
            return "";
        }

        if (read == -1) {
            System.out.println("Nothing was there to be read, closing connection");
            currentClientChannel.close();
            key.cancel();
            return "";
        }
        readBuffer.flip();
        byte[] data = new byte[1000];
        readBuffer.get(data, 0, read);
        rcvMsg =  new String(data);
        
        System.out.println("Received: " + rcvMsg);
        return rcvMsg;
    }
	

    public static void sendMessage(String msg) {
		try {
			if (currentClientChannel == null) {
				return;
			}
			if (currentClientChannel.isConnected()) {
				currentClientChannel.write(ByteBuffer.wrap(msg.getBytes()));
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

    public static String receiveMessage() {
		if (currentClientChannel == null) {
			return "";
		}
		if (currentClientChannel.isConnected()) {
			String tempMsg = rcvMsg;
			rcvMsg = "";
			return tempMsg;
		}else {
			return "";
		}
	}
}