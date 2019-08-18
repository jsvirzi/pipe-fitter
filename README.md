# pipe-fitter

This utility is a specialized version of SOCAT.
Specifically, it attaches a serial port (for example) and establishes bidirectional communication with a socket, implementing TCP/IP protocol.
Normally, this side of the connection runs as a server.

Elsewhere, or on the same machine, another process connects to the server.
The socket appears as the serial port on the other machine.
All data sent to the socket appears at the serial port.
Whereas all data read from the serial port is sent directly to the socket,
the data from the socket is read via a queue.
Robust data acquisition typically demands the serial port (or socket) to be serviced in a different thread.
The data is best collected through an asynchronous queue.
More specifically, the queue implemented is "pool_queue".
It's written in "C", and is friendly for embedded applications.
Extremely lightweight, there are no getter/setter functions.
The data is simply read from the queue tail.

# on Android, run server side first
PipeFitter -file /sdcard/logs.txt -server 8086
# in another shell, run client side
PipeFitter -file /sdcard/logs.txt -client 127.0.0.1 8086

