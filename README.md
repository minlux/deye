# DEYE

Read data from *DEYE* micro inverter via modbus TCP and output the response as *JSON*.
You have to know the inverters *IP address* and *serial number*. 

```
./deye -a <ip-address> -s <serial-number> [--req <file>] [--res <file>]
```

- Parameter `-a` speciefies the inverters *IP address* in dotted-decimal notation
- Parameter `-s` specifies the inverters serial number (there seems to be 2 serial number, here the WiFi serial number is required)
- Parameter `--req` is optional and can be used to dump the raw request into a file
- Parameter `--res` is optional and can be used to dump the raw response into a file

Example:

```
./deye -a 192.168.178.139 -s 3942535924
```

The output looks like [this](doc/out.json).

## Build

```
mkdir build
cd build
cmake ..
make
```


## Postprocess Output

This is an example how you can upload the response to a web backend, from command line, using *curl*:
```
./deye -a 192.168.178.139 -s 3942535924 | curl -H 'Content-Type: application-json' -X POST --data @- https://minlux.de/pvmonitor/api/deye
```



## Further information

You can find further information in the files in the [doc](doc) folder.

## Related Resources

- [MODBUS APPLICATION PROTOCOL SPECIFICATION V1.1b](https://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b.pdf)
- https://github.com/kbialek/deye-inverter-mqtt
- https://github.com/StephanJoubert/home_assistant_solarman
- https://github.com/jlopez77/DeyeInverter.git
