A DeviceStateServer takes input directly from a file, or alternately is passed
input through a member function call, which represents the state of some
device. Currently, the only state represented is a physical 2D position. This
position is made available for polling, which is done by a DeviceStateClient
which requests the current position.

For an example test run, CD to the directory where this README.txt file was
found, and run "make". Now run the following commands:

    (1)     make

    (2)     ./DeviceStateServer -file-input position_file.txt &
    
            This starts up a server which reads subsequent positions from
            "position_file.txt", where each line represents a 2D position.
            
    (3)     ./DeviceStateClient

            This starts up a basic client program which polls the current
            state (position) from the server. As implied above, when the server
            is reading from an input *file*, the server will automatically
            update its current state to the subsequent line. Because the
            client program is set-up to poll every second, a new line will
            be printed by the client each second.

For a more interesting example, make the parent directory:

    (1)     cd .. ; touch Dependencies ; make ; cd -

    (2)     ./DeviceStateServer -file-input position_file.txt &

    (3)     cd ..; ./basicflock ; cd -
    
            Immediately after basicflock starts, press space and then 5 to
            start the simulation and display the bounding box. Unfortunately,
            the position starts updating soon after the program starts, so
            the sooner the simulation is started, the more updates will occur
            before the end of "position_file.txt" is reached.

For manual control, try:

    (1)     Start a new terminal and run:     ./DeviceStateServer -manual-input
    
    (2)     In the original terminal, run:    cd ..; ./basicflock ; cd -

    (3)     Type coordinates in the new terminal in which the server is running to
            update the position (the bounding box is set-up with bounds (-50, 50)
            for both X and Y axes) - quit with CTRL+C.
