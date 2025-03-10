import 'dart:convert';
import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:flutter_joystick/flutter_joystick.dart';

/*
Note: used flutter_joystick package from https://pub.dev/packages/flutter_joystick#joystick
Also used ChatGPT to reference how to transmit float through BLE
*/

// define UUIDs as constants - these should match the Arduino code
const String serviceUUID = "00000000-5EC4-4083-81CD-A10B8D5CF6EC";
const String characteristicUUID = "00000001-5EC4-4083-81CD-A10B8D5CF6EC";

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title});

  final String title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  final _ble = FlutterReactiveBle();

  double _x = 0;
  double _y = 0;
  JoystickMode _joystickMode = JoystickMode.all;


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
  }

  // when terminating cancel all the subscriptions
  @override
  void dispose() {
    _notifySub?.cancel();
    _connectSub?.cancel();
    _scanSub?.cancel();
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

  Future<void> _sendCommand(double turn, double forward) async {
    
    if (_writeCharacteristic != null) {
        final ByteData data = ByteData(8);
        data.setFloat32(0, turn, Endian.little); // First 4 bytes: X-coordinate
        data.setFloat32(4, forward, Endian.little); // Next 4 bytes: Y-coordinate

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

                Joystick(
                  mode: _joystickMode,
                  listener: (details) {
                    setState(() {
                      _x = double.parse(details.x.toStringAsFixed(2));
                      _y = double.parse(details.y.toStringAsFixed(2));


                    });
                    _sendCommand(_x,_y);
                  },
                ),
                const SizedBox(height: 10),
                Text('X: $_x'),
                const SizedBox(height: 10),
                Text('Y: $_y'),
                /*
                // Joystick Buttons
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    ElevatedButton(
                      onPressed:
                          _isConnected ? () => _sendCommand('FORWARD') : null,
                      child: const Icon(Icons.arrow_upward),
                    ),
                  ],
                ),
                const SizedBox(height: 10),
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    ElevatedButton(
                      onPressed:
                          _isConnected ? () => _sendCommand('LEFT') : null,
                      child: const Icon(Icons.arrow_back),
                    ),
                    const SizedBox(width: 10),
                    ElevatedButton(
                      onPressed:
                          _isConnected ? () => _sendCommand('BACK') : null,
                      child: const Icon(Icons.arrow_downward),
                    ),
                    const SizedBox(width: 10),
                    ElevatedButton(
                      onPressed:
                          _isConnected ? () => _sendCommand('RIGHT') : null,
                      child: const Icon(Icons.arrow_forward),
                    ),
                  ],
                ),
                const SizedBox(height: 20),
                

                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    ElevatedButton(
                      onPressed: _isConnected ? () => _sendCommand('A') : null,
                      child: const Text('Send A'),
                    ),
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