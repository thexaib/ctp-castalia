Installation instructions:
--------------------------
We assume Castalia 3.0 and OMNET++ 4 being correctly installed.
From the home of castalia (CASTALIA_HOME) do the following:

1) Copy CtpTest in CASTALIA_HOME/Simulations
2) Copy ctpTestingApplication CASTALIA_HOME/src/node/application/
3) Copy ctpNoe in CASTALIA_HOME/src/node/communication/routing/
4) Copy cc2420Mac in CASTALIA_HOME/src/node/communication/mac/
5) make clean
6) ./makemake
7) make

Run a test:
------------
Go in CASTALIA_HOME/Simulations/CtpTest and run ../../bin/Castalia -c General

See the results:
----------------
From CASTALIA_HOME/Simulations/CtpTest run ../../bin/CastaliaResults -i [outputfile].txt
Statistics at application level -> "My Stats"  (note: the DDR is just the number of packets received, not the actual DDR)
Statistics from ctp: Ctp Data and Ctp Beacons
