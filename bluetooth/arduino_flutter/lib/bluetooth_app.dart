import 'dart:convert';
import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
//import 'package:flutter_joystick/flutter_joystick.dart';

/*
Note: used flutter_joystick package from https://pub.dev/packages/flutter_joystick#joystick
UPDATE MAR 25: NOT USING JOYSTICK
Also used ChatGPT to reference how to transmit float through BLE
*/

// define UUIDs as constants - these should match the Arduino code
const String serviceUUID = "fc096266-ad93-482d-928c-c2560ea93a4e";
const String characteristicUUID = "9ff0183d-6d83-4d05-a10e-55c142bee2d1";

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title});

  final String title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  final _ble = FlutterReactiveBle();

  double num1 = 0;
  double num2 = 0;
  double num3 = 0;
  double lastInput1 = 0;
  double lastInput2 = 0;
  double lastInput3 = 0;


  final TextEditingController myController1 = TextEditingController();
  final TextEditingController myController2 = TextEditingController();
  final TextEditingController myController3 = TextEditingController();

  //JoystickMode _joystickMode = JoystickMode.all;


  StreamSubscription<DiscoveredDevice>?
      _scanSub; // subscribe to bluetooth scanning stream
  StreamSubscription<ConnectionStateUpdate>?
      _connectSub; // subscribe to bluetooth connection stream
  StreamSubscription<List<int>>? _notifySub;

  List<DiscoveredDevice> _devices = [];
  String? _selectedDeviceId; // will hold the device ID selected for connection
  String?
      _selectedDeviceName; // will hold the device name selected for connection
  var _stateMessage = 'Scanning...'; // displays app status
  QualifiedCharacteristic? _writeCharacteristic;

  bool _isConnected = false; // flag to indicate connection

  // on initialization scan for devices
  @override
  void initState() {
    super.initState();
    _scanSub = _ble.scanForDevices(withServices: []).listen(_onScanUpdate);
    Future.delayed(Duration(seconds: 10), () {
      _scanSub?.cancel(); // Stop scanning after 10 seconds
    });
  }

  // when terminating cancel all the subscriptions
  @override
  void dispose() {
    _notifySub?.cancel();
    _connectSub?.cancel();
    _scanSub?.cancel();
    myController1.dispose();
    myController2.dispose();
    myController3.dispose();
    super.dispose();
  }

  // update devices that found with "BLE" in their name
  void _onScanUpdate(DiscoveredDevice d) {
    if (d.name.contains("BLE") &&
        !_devices.any((device) => device.id == d.id)) {
      setState(() {
        _devices.add(d);
      });
    }
  }

  // Connect to the devices that was selected by user
  void _connectToDevice() {
    if (_selectedDeviceId != null) {
      setState(() {
        _stateMessage = 'Connecting to $_selectedDeviceName...';
      });

      _connectSub = _ble.connectToDevice(id: _selectedDeviceId!).listen(
        (update) {
          if (update.connectionState == DeviceConnectionState.connected) {
            setState(() {
              _stateMessage = 'Connected to $_selectedDeviceName!';
              _isConnected = true;
            });
            _onConnected(_selectedDeviceId!);
          }  else if (update.connectionState == DeviceConnectionState.disconnected) {
          // Handle unexpected disconnections
          setState(() {
            _stateMessage = 'Disconnected from $_selectedDeviceName.';
            _isConnected = false;
            _writeCharacteristic = null;
          });

        }
      },
        onError: (error) {
          setState(() {
            _stateMessage = 'Connection error: $error';
          });
        },
      );
    }
  }

  // Handle disconnection
  void _disconnectFromDevice() {
    try {
      if (_notifySub != null) {
        _notifySub?.cancel();
        _notifySub = null;
      }

      if (_connectSub != null) {
        _connectSub?.cancel();
        _connectSub = null;
      }

      setState(() {
        _isConnected = false;
        _stateMessage = 'Disconnected from $_selectedDeviceName.';
        _writeCharacteristic = null;
      });
    } catch (e) {
      setState(() {
        _stateMessage = 'Error during disconnection: $e';
      });
    }
  }

  void _onConnected(String deviceId) {
    final characteristic = QualifiedCharacteristic(
      deviceId: deviceId,
      serviceId: Uuid.parse(serviceUUID), // Use the constant here
      characteristicId: Uuid.parse(characteristicUUID), // Use the constant here
    );

    _writeCharacteristic = characteristic;

    _notifySub = _ble.subscribeToCharacteristic(characteristic).listen((bytes) {
      setState(() {
        _stateMessage = 'Data received: ${Utf8Decoder().convert(bytes)}';
      });
    });
  }

  Future<void> _sendCommand(int turnCommand, double p, double i, double d) async {
    
    if (_writeCharacteristic != null) {
        final ByteData data = ByteData(1);
        //data.setFloat32(0, turn, Endian.little); // First 4 bytes: X-coordinate
        //data.setFloat32(4, forward, Endian.little); // Next 4 bytes: Y-coordinate
        data.setInt8(0, turnCommand);
        //data.setFloat32(0, p, Endian.little); // Next 4 bytes: p
        //data.setFloat32(4, i, Endian.little); // Next 4 bytes: i
        //data.setFloat32(8, d, Endian.little); // Next 4 bytes: d

        final List<int> sendData = data.buffer.asUint8List();
      try {
        await _ble.writeCharacteristicWithResponse(

          _writeCharacteristic!,
          value: sendData,
        );
        setState(() {
          _stateMessage = "Command sent!";
        });
      } catch (e) {
        setState(() {
          _stateMessage = "Error sending command: $e";
        });
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Text(widget.title),
      ),
      body: Column(
        children: [
          Container(
            padding: const EdgeInsets.all(16.0),
            color: Colors.grey[300],
            width: double.infinity,
            child: Text(
              _stateMessage,
              style: Theme.of(context).textTheme.titleMedium,
              textAlign: TextAlign.center,
            ),
          ),
          if (_devices.isNotEmpty)
            Padding(
              padding: const EdgeInsets.all(16.0),
              child: DropdownButton<String>(
                isExpanded: true,
                hint: const Text("Select a BLE Device"),
                value: _selectedDeviceId,
                items: _devices.map((device) {
                  return DropdownMenuItem(
                    value: device.id,
                    child: Text(device.name),
                  );
                }).toList(),
                onChanged: (value) {
                  setState(() {
                    _selectedDeviceId = value;
                    _selectedDeviceName = _devices
                        .firstWhere((device) => device.id == value)
                        .name;
                  });
                },
              ),
            ),
          if (!_isConnected)
            ElevatedButton(
              onPressed: _selectedDeviceId != null ? _connectToDevice : null,
              child: const Text('Connect'),
            ),
          if (_isConnected)
            ElevatedButton(
              onPressed: _disconnectFromDevice,
              child: const Text('Disconnect'),
            ),

          
          
          Expanded(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                /*
                Joystick(
                  mode: _joystickMode,
                  listener: (details) {
                    setState(() {
                      _x = double.parse(details.x.toStringAsFixed(2));
                      _y = double.parse(details.y.toStringAsFixed(2));


                    });
                    //_sendCommand(_x,_y, num1, num2, num3);
                  },
                ),
                const SizedBox(height: 10),
                Text('X: $_x'),
                const SizedBox(height: 10),
                Text('Y: $_y'),
                */

                
                // Joystick Buttons
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                  SizedBox(
                    width: 100,
                    height:100,
                    child: 
                    ElevatedButton(
                      onPressed:
                          _isConnected ? () => _sendCommand(1,2,3,3) : null,
                      child: const Icon(Icons.arrow_upward),
                    ),
                  )
                    
                  ],
                ),
                const SizedBox(height: 10),
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    SizedBox(
                      width: 100,
                      height:100,
                      child: 
                      ElevatedButton(
                        onPressed:
                            _isConnected ? () => _sendCommand(2,2,3,3) : null,
                        child: const Icon(Icons.arrow_back),
                      ),
                   ),
                   const SizedBox(width: 10),
                    SizedBox(
                      width: 100,
                      height:100,
                      child: 
                      ElevatedButton(
                        onPressed:
                            _isConnected ? () => _sendCommand(0,2,3,3) : null,
                        child: const Icon(Icons.stop_outlined),
                      ),
                   ),
                    const SizedBox(width: 10),
                    SizedBox(
                      width: 100,
                      height:100,
                      child: 
                      ElevatedButton(
                        onPressed:
                            _isConnected ? () => _sendCommand(3,2,3,3) : null,
                        child: const Icon(Icons.arrow_forward),
                      ),
                   ),
                  ],
                ),
                const SizedBox(height: 20),
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    SizedBox(
                      width: 100,
                      height: 100,
                      child: 
                      ElevatedButton(
                        onPressed:
                            _isConnected ? () => _sendCommand(-1,2,3,3) : null,
                        child: const Icon(Icons.arrow_downward),
                      ),
                   ),
                  ],
                ), 
                

                /*
                Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    
                    Text('LAST INPUT: P: $lastInput1, I: $lastInput2, D: $lastInput3'),
                    const SizedBox(height: 10),
                    Text('CURRENT INPUT: P: $num1, I: $num2, D: $num3'),
 
                    TextFormField(
 
                      controller: myController1,
                      keyboardType: TextInputType.number,
 
                     ),
                     SizedBox(height: 20),
 
                     TextFormField(
 
                       controller: myController2,
                       keyboardType: TextInputType.number,
 
                     ),
                     SizedBox(height: 20),
 
                     TextFormField(
 
                       controller: myController3,
                       keyboardType: TextInputType.number,
 
                     ),
                     SizedBox(height: 20),
                    
                    ElevatedButton(
                      onPressed: (){
                        lastInput1 = num1;
                        lastInput2 = num2;
                        lastInput3 = num3;

                        num1 = double.tryParse(myController1.text) ?? 0;
                        num2 = double.tryParse(myController2.text) ?? 0;
                        num3 = double.tryParse(myController3.text) ?? 0;
                        if(_isConnected){
                          _sendCommand(0, 0, num1, num2, num3);
                        }
                        
                      },
                      child: const Text('Send PID'),
                    ),
                    
                    /*
                    const SizedBox(width: 10),
                    ElevatedButton(
                      onPressed: _isConnected ? () => _sendCommand('B') : null,
                      child: const Text('Send B'),
                    ),
                    const SizedBox(width: 10),
                    ElevatedButton(
                      onPressed: _isConnected ? () => _sendCommand('C') : null,
                      child: const Text('Send C'),
                    ),
                    */
                  ],
                ),
                */
                
                
              ],
            ),
          ),
          // **************** end of command buttons ****************
        ],
      ),
    );
  }
}