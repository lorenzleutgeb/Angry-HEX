/*******************************************************************************
 * Angry-HEX - an artificial player for Angry Birds based on declarative knowledge bases
 * Copyright (C) 2012-2016 Francesco Calimeri, Michael Fink, Stefano Germano, Andreas Humenberger, Giovambattista Ianni, Christoph Redl, Daria Stepanova, Peter Schueller, Andrea Tucci, Anton Wimmer.
 *
 * This file is part of Angry-HEX.
 *
 * Angry-HEX is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * Angry-HEX is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License along with this program. If not, see <http://www.gnu.org/licenses/>.
 *******************************************************************************/
package angryhexclient;

import ab.demo.other.ClientActionRobotJava;
import angryhexclient.util.LogFormatter;
import angryhexclient.util.OutConsoleHandler;

import java.util.logging.Level;
import java.util.logging.Logger;

public class HexMainEntry {

	private static Logger Log = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);

	/**
	 * This is the main entry of the agent. It creates the server communicator
	 * It creates and starts our agent (HexAgent)
	 *
	 * run the jar: java -jar file.jar [127.0.0.1 [-level 3]]
	 */
	public static void main(final String args[]) {

		// setup logger
		// TODO distinguish debug levels
		final OutConsoleHandler ch = new OutConsoleHandler();
		ch.setFormatter(new LogFormatter());
		ch.setLevel(Level.ALL);
		HexMainEntry.Log.addHandler(ch);
		HexMainEntry.Log.setLevel(Level.ALL);

		HexMainEntry.Log.info("starting HexMainEntry main");
		try {

			// check arguments for ip address and initial level
			byte initialLevel = 1;
			String ipaddress = "127.0.0.1";
			if (args.length > 0) {
				ipaddress = args[0];

				if (args.length > 2 && args[1].equals("-level"))
					initialLevel = (byte) Integer.parseInt(args[2]);
			}

			// setup agent
			try {
				// create the object to communicate with the server
				HexMainEntry.Log.info("creating HexActionRobot at '" + ipaddress + "'");
				final ClientActionRobotJava ar = new ClientActionRobotJava(ipaddress);

				// create the agent
				HexMainEntry.Log.info("creating HexAgent with level " + Integer.toString(initialLevel));
				final HexAgent ha = new HexAgent(ar, initialLevel);

				// start the agent
				new Thread(ha).start();

			} catch (final Exception e) {
				HexMainEntry.Log.log(Level.SEVERE, "Cannot init agent: " + e.getMessage(), e);
			}

		} catch (final Exception e) {
			HexMainEntry.Log.log(Level.SEVERE, "General Error: " + e.getMessage(), e);
		}
	}
}
