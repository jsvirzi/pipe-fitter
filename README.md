# on Android, run server side first
PipeFitter -file /sdcard/logs.txt -server 8086
# in another shell, run client side
PipeFitter -file /sdcard/logs.txt -client 127.0.0.1 8086

