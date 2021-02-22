package com.rajant.bcapi.watchstate;

import com.rajant.bcapi.protos.BCAPIProtos;
import com.rajant.bcapi.protos.StateProtos;
import com.rajant.bcapi.protos.BCAPIProtos.BCMessage;
import com.rajant.bcapi.protos.BCAPIProtos.BCMessage.Builder;
import com.rajant.bcapi.protos.ConfigProtos.Config.Wireless;
import com.rajant.bcapi.protos.ConfigProtos.Config.Wireless.AP;
import com.rajant.bcapi.protos.ConfigProtos.Config.Wireless.APOrBuilder;
import com.rajant.bcapi.protos.ConfigProtos.Config.WirelessOrBuilder;
import com.rajant.bcapi.protos.StateProtos.State;
import com.rajant.bcapi.session.BreadCrumbSession;
import com.rajant.bcapi.session.BreadCrumbSessionAdapter;
import com.rajant.bcapi.session.BreadCrumbSessionStateException;
import com.rajant.bcapi.session.Role;
import com.rajant.bcapi.session.SimpleAuthenticator;
import com.rajant.bcapi.session.sync.SyncBreadCrumbSessionFactory;
import com.rajant.bcapi.util.proto.ProtoUtils;
import com.rajant.bcapi.logging.Level;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;



/**
 * An example of watching BreadCrumb state and merging in watch responses (deltas).
 */

public class WatchStateApp {
    private static final Logger log = LoggerFactory.getLogger(WatchStateApp.class);

    /**
     * Stop after four messages
     */
    private static int maxCount = 4;
    private static int msgCount = 0;
    private static AtomicLong sequence = new AtomicLong(0);

    /**
     * Latest BreadCrumb state
     */
    private static BCMessage bcMessage;

    public static void main(String[] args) throws IOException {
        com.rajant.bcapi.logging.Config.logToConsole();
        com.rajant.bcapi.logging.Config.setLevel(Level.INFO);
        com.rajant.bcapi.logging.Config.setLevel(LoggerFactory.getLogger("com.rajant.bcapi.session"), Level.INFO);
        com.rajant.bcapi.logging.Config.setLevel(LoggerFactory.getLogger("com.rajant.bcapi.session.sync.BCAPIMessageOutputStream"), Level.INFO);

        /**
         * Authentication information and BreadCrumb IP address (ipv4 or ipv6) are passed as arguments
         */
        String password = "breadcrumb-admin";
        String userAgent = "example app";
        if (args.length != 1) {
            log.error("YOU HAVE TO ENTER ONLY ONE IP");
            System.exit(1);
        }
        String myBreadCrumb = args[0];
        //log.info("TARGET IP:{}\n", myBreadCrumb);
        //System.out.println(myBreadCrumb);
        // String myBreadCrumb = "10.229.221.1";


        /**
         * A user agent and password may be required to login to BreadCrumbs and access protobuf information
         */
        final SimpleAuthenticator auth = new SimpleAuthenticator(Role.ADMIN, password, userAgent);
        final SyncBreadCrumbSessionFactory factory = new SyncBreadCrumbSessionFactory();
        final BreadCrumbSession session = factory.createSession("session");

        /**
         * Authenticate the session and register watchers

         After the BreadCrumb session has authenticated
         the code in the following init() function will be run
         In this instance we are setting up several watchers.
         */

        session.setAuthInitializer(new BreadCrumbSession.AuthInitializer() {
            public void init(BreadCrumbSession session, BCMessage challenge, Builder mb) {
                BCMessage.WatchRequest.Builder wrb = BCMessage.WatchRequest.newBuilder();
                // wrb.addWatchObjectBuilder().setMessagePath("configuration.active.general").setInterval(0);
                // wrb.addWatchObjectBuilder().setMessagePath("system.uptime").setInterval(1);
                wrb.addWatchObjectBuilder().setMessagePath("wireless").setInterval(1);
                mb.setWatchRequest(wrb);
                mb.setSequenceNumber(sequence.incrementAndGet());
            }
        });

        /**
         * Handle responses from the BreadCrumb
         */
        session.addBreadCrumbSessionListener(new BreadCrumbSessionAdapter() {

            /**
             * A BCAPI massage was received from the breadcrumb
             * @param session
             * @param message
             */
            @Override
            public void messageReceived(BreadCrumbSession session, BCMessage message) {
                /**
                 * Was the message a watchResponse and does it contain deltas
                 */
                if (message.hasWatchResponse() && message.getWatchResponse().getIsDelta()){

                    msgCount++;
                    // log.info("received message [{}]:\n{}\n",msgCount, message);
                    // log.info("{}\n", message);
                    System.out.println(message);

                    /**
                     * Since we care about State make sure the watch response contains State
                     */
                    if (bcMessage != null && message.getWatchResponse().hasState()){
                        /**
                         * Apply changes to current State and rebuild message
                         */
                        // log.info("Updating state");
                        State.Builder currState = bcMessage.getState().toBuilder();
                        ProtoUtils.merge(currState, message.getWatchResponse().getState());
                        bcMessage =  bcMessage.toBuilder().setState(currState).build();
                    }
                }
                /**
                 *  Don't listen forever
                 */
                /*
                if (msgCount >= maxCount){
                    session.close();
                }
                */
            }
        });


        try {
            session.start(new InetSocketAddress(myBreadCrumb, 2300), auth);
            session.waitForAuthentication();

            /**
             * Manually request a full copy of current State by sending a
             message containing empty state
             */
            Builder b = session.newMessage();
            b.setState(State.newBuilder());

            bcMessage = session.sendAndReceive(b.build());
            //System.out.println("SUCCESS");
        }
        catch (Throwable e) {
            log.error("failed", e);
        }
    }
}
