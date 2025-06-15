#include <Bluepad32.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// PCF8574 Adresse (üblich ist 0x20, kann aber je nach Hardwareeinstellung variieren)
const int PCF8574_ADDR = 0x20;  

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            u8g2.clearBuffer();          // clear the internal memory
            u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
            u8g2.drawStr(0,55,"Gamepad found");  // write something to the internal memory
            u8g2.sendBuffer();          // transfer internal memory to the display
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            u8g2.clearBuffer();          // clear the internal memory
            u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
            u8g2.drawStr(0,55,"Gamepad Lost");  // write something to the internal memory
            u8g2.sendBuffer();          // transfer internal memory to the display
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, "
        "misc: 0x%02x\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->miscButtons()  // bitmask of pressed "misc" buttons
    );
}



// Funktion zum Setzen einzelner Pins
uint8_t setPCFPin(uint8_t currentState, uint8_t pin, bool state) {
  if(state)
    currentState |= (1 << pin);   // Pin auf HIGH setzen
  else
    currentState &= ~(1 << pin);  // Pin auf LOW setzen
  return currentState;
}

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    uint8_t currentState = 0b11111111;
    int yo = 15;
    if (ctl->buttons() & 0x02) {
      Serial.println("A");
      u8g2.drawStr(120,20+yo,"A");
      currentState = setPCFPin(currentState, 0, LOW); // Test
    }

    if (ctl->buttons() & 0x01) {
      Serial.println("B");
      u8g2.drawStr(110,30+yo,"B");
      currentState = setPCFPin(currentState, 7, LOW); //Button
    }
    if (ctl->buttons() & 0x04) {
      Serial.println("Y");
      u8g2.drawStr(100,20+yo,"Y");
      currentState = setPCFPin(currentState, 1, LOW); //Coin
    }

    if (ctl->buttons() & 0x08) {
      Serial.println("X");
      u8g2.drawStr(110,10+yo,"X");
      currentState = setPCFPin(currentState, 1, LOW); //Coin
    }
    if (ctl->dpad() & 0x01) {
      Serial.println("up");
      u8g2.drawStr(10+70,10+yo,"U");
      currentState = setPCFPin(currentState, 5, LOW); //Up
    }
    if (ctl->dpad() & 0x02) {
      Serial.println("down");
      u8g2.drawStr(10+70,30+yo,"D");
      currentState = setPCFPin(currentState, 6, LOW); //Down
    }
    if (ctl->dpad() & 0x04) {
      Serial.println("right");
      u8g2.drawStr(20+70,20+yo,"R");
      currentState = setPCFPin(currentState, 4, LOW); //Right
    }
    if (ctl->dpad() & 0x08) {
      Serial.println("left");
      u8g2.drawStr(0+70,20+yo,"L");
      currentState = setPCFPin(currentState, 3, LOW); //Left
    }
    if (ctl->buttons() & 0x10) {
      Serial.println("left shoulder");
      u8g2.drawStr(7+70,0+yo,"LS");
    }
    if (ctl->buttons() & 0x20) {
      Serial.println("right shoulder");
      u8g2.drawStr(37+70,0+yo,"RS");
    }
    if (ctl->miscButtons() & 0x02) {
      Serial.println("select");
      u8g2.drawStr(0+70,40+yo,"select");
    }
    if (ctl->miscButtons() & 0x04) {
      Serial.println("start");
      u8g2.drawStr(30+70,40+yo,"start");
      currentState = setPCFPin(currentState, 2, LOW); //Start
    }
    Serial.println(currentState, BIN); 
      // Einzelne Pins setzen (Beispiel: Pin 0 und 3 auf HIGH)
    Wire.beginTransmission(PCF8574_ADDR);
    Wire.write(currentState);  // Bit 0 und 3 auf 1
    Wire.endTransmission();
    
    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    //dumpGamepad(ctl);
}


void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Wire.begin(21, 22);  // SDA, SCL
    Serial.begin(115200);
    // Alle Pins auf HIGH setzen
    Wire.beginTransmission(PCF8574_ADDR);
    Wire.write(0xFF);  // 1111 1111
    Wire.endTransmission();
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    u8g2.begin();
    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(0,55,"Searching Gamepad");  // write something to the internal memory
    u8g2.sendBuffer();          // transfer internal memory to the display

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    // BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        u8g2.clearBuffer();          // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
        u8g2.drawStr(0,10,"Gamepad");  // write something to the internal memory
        u8g2.drawStr(0,20,"Host");  // write something to the internal memory
        processControllers();
        u8g2.sendBuffer();          // transfer internal memory to the display
    }

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);

    delay(150);
}
