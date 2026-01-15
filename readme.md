{
  "version": 1,
  "author": "Gemini Rework",
  "editor": "wokwi",
  "parts": [
    {
      "type": "board-esp32-devkit-c-v4",
      "id": "esp",
      "top": -19.2,
      "left": -4.76,
      "attrs": { "builder": "esp-idf" }
    },
    {
      "type": "wokwi-led",
      "id": "ledGreen",
      "top": 111.6,
      "left": 282.2,
      "attrs": { "color": "red" }
    },
    {
      "type": "wokwi-led",
      "id": "ledRed",
      "top": 44.4,
      "left": 282.2,
      "attrs": { "color": "yellow" }
    },
    {
      "type": "wokwi-pushbutton",
      "id": "button",
      "top": -115.8,
      "left": 233.4,
      "rotate": 90,
      "attrs": { "color": "red" }
    },
    {
      "type": "wokwi-resistor",
      "id": "r1",
      "top": 157.85,
      "left": 190.6,
      "rotate": 180,
      "attrs": { "value": "220" }
    },
    {
      "type": "wokwi-resistor",
      "id": "r2",
      "top": 90.65,
      "left": 190.6,
      "rotate": 180,
      "attrs": { "value": "220" }
    },
    {
      "type": "wokwi-hc-sr04",
      "id": "ultrasonic1",
      "top": -171.3,
      "left": -42.5,
      "attrs": { "distance": "96" }
    },
    {
      "type": "wokwi-led",
      "id": "ledYellow",
      "top": -22.8,
      "left": 282.2,
      "attrs": { "color": "green" }
    },
    {
      "type": "wokwi-resistor",
      "id": "r3",
      "top": 23.45,
      "left": 190.6,
      "rotate": 180,
      "attrs": { "value": "220" }
    },
    {
      "type": "wokwi-text",
      "id": "label_estop",
      "top": -153.6,
      "left": 182.4,
      "attrs": { "text": "Emergency Brake / Restart" }
    },
    {
      "type": "wokwi-text",
      "id": "label_sensor",
      "top": -201.6,
      "left": -19.2,
      "attrs": { "text": "Proximity Sensor" }
    },
    {
      "type": "wokwi-text",
      "id": "label_clear",
      "top": -9.6,
      "left": 326.4,
      "attrs": { "text": "Ride Operational" }
    },
    {
      "type": "wokwi-text",
      "id": "label_power",
      "top": 57.6,
      "left": 326.4,
      "attrs": { "text": "System Power" }
    },
    {
      "type": "wokwi-text",
      "id": "label_brake",
      "top": 124.8,
      "left": 326.4,
      "attrs": { "text": "Ride Halted" }
    }
  ],
  "connections": [
    [ "esp:TX", "$serialMonitor:RX", "", [] ],
    [ "esp:RX", "$serialMonitor:TX", "", [] ],
    [ "ultrasonic1:VCC", "esp:VIN", "red", [ "v0" ] ],
    [ "r3:2", "esp:GND.2", "black", [ "h-46.8", "v-19.2" ] ],
    [ "r2:2", "esp:GND.2", "black", [ "h-46.8", "v-86.4" ] ],
    [ "r1:2", "esp:GND.2", "black", [ "h-46.8", "v-153.6" ] ],
    [ "ledYellow:C", "r3:1", "green", [ "v0" ] ],
    [ "ledRed:C", "r2:1", "gold", [ "v0" ] ],
    [ "r1:1", "ledGreen:C", "red", [ "v0", "h76.8" ] ],
    [ "ledRed:A", "esp:5", "gold", [ "v19.2", "h-172.8", "v-9.6" ] ],
    [ "ledGreen:A", "esp:4", "red", [ "v19.2", "h-172.8", "v-48" ] ],
    [ "ledYellow:A", "esp:19", "green", [ "v19.2", "h-172.8", "v38.4" ] ],
    [ "button:2.r", "esp:GND.2", "black", [ "v9.8", "h-115.4", "v57.6" ] ],
    [ "button:1.r", "esp:18", "red", [ "v19.4", "h-124.8", "v124.8" ] ],
    [ "ultrasonic1:VCC", "esp:5V", "white", [ "v48", "h-48", "v211.2" ] ],
    [ "ultrasonic1:TRIG", "esp:17", "violet", [ "v48", "h76.4", "v134.4", "h9.6" ] ],
    [ "ultrasonic1:ECHO", "esp:16", "blue", [ "v38.4", "h76", "v153.6" ] ],
    [ "ultrasonic1:GND", "esp:GND.2", "black", [ "v28.8", "h85.2", "v57.6" ] ]
  ],
  "dependencies": {}
}