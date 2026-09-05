/* ============================================================
   HYDROPONICS PERISTALTIC PUMP TESTER
   ============================================================

   Purpose
   -------
   Manually test and identify each peristaltic pump over I2C
   before running the full automatic hydroponics controller.

   IMPORTANT:
   This address mapping MATCHES hydra_main_fixed.ino:

       0x32 = pH Down
       0x33 = Nutrient A
       0x34 = Nutrient B
       0x35 = EC Down / Water

   There is NO pH-Up pump in the main controller.

   Recommended first test
   ----------------------
   Put ALL pump inlet tubes into plain water.
   Put each outlet tube into a separate container.

   Then test ONE pump at a time from Serial Monitor:

       dose phdn 2
       dose nuta 2
       dose nutb 2
       dose ecdn 2

   Label each physical pump after confirming which one runs.

   Serial Monitor:
       Baud:       9600
       Line ending: Newline

   ============================================================ */


#include <Arduino.h>
#include <Wire.h>


/* ============================================================
   I2C PUMP ADDRESSES

   These MUST remain consistent with the main controller.
   ============================================================ */

#define PH_DOWN_ADDR     0x32   // pH Down solution
#define NUTRIENT_A_ADDR  0x33   // Nutrient Part A
#define NUTRIENT_B_ADDR  0x34   // Nutrient Part B
#define EC_DOWN_ADDR     0x35   // Plain water / EC dilution


/* ============================================================
   TEST SAFETY LIMITS
   ============================================================ */

// Maximum volume allowed in a single "dose" command.
static const float MAX_TEST_ML = 50.0f;

// Maximum runtime allowed with "time" or "prime".
static const unsigned long MAX_TEST_MS = 60000UL;

// Serial Monitor baud rate.
static const unsigned long SERIAL_BAUD = 9600;


/* ============================================================
   PUMP DESCRIPTION TABLE
   ============================================================

   Commands use these short names:

       phdn = pH Down
       nuta = Nutrient A
       nutb = Nutrient B
       ecdn = EC Down / water
   ============================================================ */

struct PumpDesc {
  const char *name;
  const char *description;
  uint8_t address;
};


PumpDesc PUMPS[] = {

  {
    "phdn",
    "pH Down",
    PH_DOWN_ADDR
  },

  {
    "nuta",
    "Nutrient A",
    NUTRIENT_A_ADDR
  },

  {
    "nutb",
    "Nutrient B",
    NUTRIENT_B_ADDR
  },

  {
    "ecdn",
    "EC Down / Water",
    EC_DOWN_ADDR
  }
};


static const size_t NUM_PUMPS =
  sizeof(PUMPS) / sizeof(PUMPS[0]);


/* ============================================================
   HELP TEXT
   ============================================================ */

const char *HELP_TEXT =

  "\n"
  "====================================================\n"
  " Hydroponics Peristaltic Pump Tester\n"
  "====================================================\n"
  "\n"
  "Commands:\n"
  "\n"
  "  help\n"
  "      Show this help screen.\n"
  "\n"
  "  list\n"
  "      Show pump names, roles and I2C addresses.\n"
  "\n"
  "  dose <pump> <mL>\n"
  "      Command a calibrated volume.\n"
  "      Example: dose nuta 2\n"
  "\n"
  "  time <pump> <ms>\n"
  "      Run a pump by time, if supported by its firmware.\n"
  "      Example: time nuta 2000\n"
  "\n"
  "  prime <pump> <ms>\n"
  "      Same as 'time'. Intended for priming tubing.\n"
  "      Example: prime phdn 3000\n"
  "\n"
  "  stop <pump>\n"
  "      Send STOP to one pump, if supported.\n"
  "      Example: stop nutb\n"
  "\n"
  "  stop all\n"
  "      Send STOP to all four pumps.\n"
  "\n"
  "  testall\n"
  "      Dose 1 mL from each pump sequentially.\n"
  "      USE WITH PLAIN WATER ONLY while identifying pumps.\n"
  "\n"
  "Pump names:\n"
  "\n"
  "  phdn = pH Down          @ 0x32\n"
  "  nuta = Nutrient A       @ 0x33\n"
  "  nutb = Nutrient B       @ 0x34\n"
  "  ecdn = EC Down / Water  @ 0x35\n"
  "\n"
  "Recommended identification sequence:\n"
  "\n"
  "  dose phdn 2\n"
  "  dose nuta 2\n"
  "  dose nutb 2\n"
  "  dose ecdn 2\n"
  "\n";


/* ============================================================
   I2C COMMUNICATION
   ============================================================ */

/*
   Send one ASCII command to a pump controller.

   Returns:
       true  = I2C transmission was acknowledged
       false = I2C transmission failed

   Important:
   An I2C acknowledgement tells us the slave received the
   transmission. It does NOT prove that liquid physically moved.
*/
bool i2cSend(uint8_t address, const char *command) {

  Wire.beginTransmission(address);

  while (*command) {
    Wire.write(*command++);
  }

  const uint8_t error = Wire.endTransmission();

  if (error != 0) {

    Serial.print(F("ERROR: I2C transmission failed to 0x"));
    Serial.print(address, HEX);

    Serial.print(F(" | error code "));
    Serial.println(error);

    return false;
  }

  delay(10);

  return true;
}


/* ============================================================
   PUMP COMMANDS
   ============================================================ */


/*
   Dose a calibrated volume.

   This uses the SAME command format as the main hydroponics
   controller:

       D,<millilitres>

   Example:

       D,2.00
*/
bool pumpDose_mL(uint8_t address, float mL) {

  if (mL <= 0.0f) {

    Serial.println(
      F("ERROR: Dose volume must be greater than 0 mL.")
    );

    return false;
  }


  if (mL > MAX_TEST_ML) {

    Serial.print(F("Requested dose exceeds safety limit. "));
    Serial.print(F("Clamping to "));
    Serial.print(MAX_TEST_ML, 1);
    Serial.println(F(" mL."));

    mL = MAX_TEST_ML;
  }


  String command =
    "D," + String(mL, 2);


  return i2cSend(
    address,
    command.c_str()
  );
}


/*
   Run a pump for a specified number of milliseconds.

   Command:

       T,<milliseconds>

   NOTE:
   The main automatic controller does NOT use this command.

   Only use this function if your individual pump-controller
   firmware supports the T command.
*/
bool pumpRun_timeMs(
  uint8_t address,
  unsigned long ms
) {

  if (ms == 0) {

    Serial.println(
      F("ERROR: Runtime must be greater than 0 ms.")
    );

    return false;
  }


  if (ms > MAX_TEST_MS) {

    Serial.print(F("Requested runtime exceeds safety limit. "));
    Serial.print(F("Clamping to "));
    Serial.print(MAX_TEST_MS);
    Serial.println(F(" ms."));

    ms = MAX_TEST_MS;
  }


  String command =
    "T," + String(ms);


  return i2cSend(
    address,
    command.c_str()
  );
}


/*
   Send a stop command.

   NOTE:
   This requires the pump-controller firmware to support "X".

   The automatic hydroponics controller does not rely on this
   command for normal calibrated dosing.
*/
bool pumpStop(uint8_t address) {

  return i2cSend(
    address,
    "X"
  );
}


/* ============================================================
   PUMP LOOKUP
   ============================================================ */

/*
   Convert a command name such as:

       nuta

   into its corresponding I2C address and description.
*/
bool lookupPump(
  const String &token,
  uint8_t &addressOut,
  const char* &nameOut,
  const char* &descriptionOut
) {

  String search = token;

  search.trim();
  search.toLowerCase();


  for (size_t i = 0; i < NUM_PUMPS; ++i) {

    if (search.equals(PUMPS[i].name)) {

      addressOut = PUMPS[i].address;

      nameOut =
        PUMPS[i].name;

      descriptionOut =
        PUMPS[i].description;

      return true;
    }
  }


  return false;
}


/* ============================================================
   DISPLAY PUMP LIST
   ============================================================ */

void printList() {

  Serial.println();
  Serial.println(F("Configured pumps:"));
  Serial.println();


  for (size_t i = 0; i < NUM_PUMPS; ++i) {

    Serial.print(F("  "));

    Serial.print(
      PUMPS[i].name
    );

    Serial.print(F("  |  "));

    Serial.print(
      PUMPS[i].description
    );

    Serial.print(F("  |  I2C 0x"));

    Serial.println(
      PUMPS[i].address,
      HEX
    );
  }


  Serial.println();
}


/* ============================================================
   TEST ALL PUMPS
   ============================================================ */

/*
   Runs a very small calibrated dose through each pump.

   IMPORTANT:
   Use plain water when using this while identifying pumps.

   Individual testing is still preferred.
*/
void quickTestAll() {

  const float TEST_ML = 1.0f;

  const unsigned long GAP_MS =
    1000UL;


  Serial.println();

  Serial.println(
    F("WARNING: testall will activate ALL FOUR pumps.")
  );

  Serial.println(
    F("Use this with plain water while identifying pumps.")
  );

  Serial.println();


  for (size_t i = 0; i < NUM_PUMPS; ++i) {

    Serial.print(F("Testing "));

    Serial.print(
      PUMPS[i].name
    );

    Serial.print(F(" ("));

    Serial.print(
      PUMPS[i].description
    );

    Serial.print(F(") @ 0x"));

    Serial.print(
      PUMPS[i].address,
      HEX
    );

    Serial.print(F(" -> "));

    Serial.print(
      TEST_ML,
      1
    );

    Serial.println(F(" mL"));


    const bool ok =
      pumpDose_mL(
        PUMPS[i].address,
        TEST_ML
      );


    if (ok) {

      Serial.println(
        F("  I2C command accepted.")
      );

    } else {

      Serial.println(
        F("  FAILED - command not acknowledged.")
      );
    }


    delay(GAP_MS);
  }


  Serial.println();

  Serial.println(
    F("testall complete.")
  );

  Serial.println();
}


/* ============================================================
   HELP
   ============================================================ */

void printHelp() {

  Serial.print(
    HELP_TEXT
  );
}


/* ============================================================
   COMMAND PARSER
   ============================================================ */

void parseLine(String line) {

  line.trim();


  if (line.length() == 0) {
    return;
  }


  /* ----------------------------------------------------------
     Split the command into:

         command argument1 argument2

     Example:

         dose nuta 2.5
  ---------------------------------------------------------- */

  String command;
  String arg1;
  String arg2;


  const int firstSpace =
    line.indexOf(' ');


  if (firstSpace < 0) {

    command = line;

  } else {

    command =
      line.substring(
        0,
        firstSpace
      );


    String remainder =
      line.substring(
        firstSpace + 1
      );


    remainder.trim();


    const int secondSpace =
      remainder.indexOf(' ');


    if (secondSpace < 0) {

      arg1 = remainder;

    } else {

      arg1 =
        remainder.substring(
          0,
          secondSpace
        );


      arg2 =
        remainder.substring(
          secondSpace + 1
        );


      arg2.trim();
    }
  }


  command.toLowerCase();


  /* ----------------------------------------------------------
     HELP
  ---------------------------------------------------------- */

  if (command == "help") {

    printHelp();

    return;
  }


  /* ----------------------------------------------------------
     LIST
  ---------------------------------------------------------- */

  if (command == "list") {

    printList();

    return;
  }


  /* ----------------------------------------------------------
     TEST ALL
  ---------------------------------------------------------- */

  if (command == "testall") {

    quickTestAll();

    return;
  }


  /* ----------------------------------------------------------
     STOP
  ---------------------------------------------------------- */

  if (command == "stop") {

    if (
      arg1.length() == 0 ||
      arg1.equalsIgnoreCase("all")
    ) {

      Serial.println(
        F("Sending STOP to all pumps...")
      );


      for (
        size_t i = 0;
        i < NUM_PUMPS;
        ++i
      ) {

        const bool ok =
          pumpStop(
            PUMPS[i].address
          );


        Serial.print(F("  "));

        Serial.print(
          PUMPS[i].name
        );

        Serial.print(F(": "));


        if (ok) {

          Serial.println(
            F("command accepted")
          );

        } else {

          Serial.println(
            F("I2C FAILED")
          );
        }
      }


      return;
    }


    uint8_t address;

    const char* pumpName;
    const char* description;


    if (
      !lookupPump(
        arg1,
        address,
        pumpName,
        description
      )
    ) {

      Serial.println(
        F("Unknown pump.")
      );

      Serial.println(
        F("Use: phdn, nuta, nutb, ecdn")
      );

      return;
    }


    const bool ok =
      pumpStop(address);


    if (ok) {

      Serial.print(F("STOP command accepted by "));
      Serial.println(description);

    } else {

      Serial.print(F("STOP FAILED for "));
      Serial.println(description);
    }


    return;
  }


  /* ----------------------------------------------------------
     DOSE
  ---------------------------------------------------------- */

  if (command == "dose") {

    if (
      arg1.length() == 0 ||
      arg2.length() == 0
    ) {

      Serial.println(
        F("Usage: dose <pump> <mL>")
      );

      Serial.println(
        F("Example: dose nuta 2")
      );

      return;
    }


    uint8_t address;

    const char* pumpName;
    const char* description;


    if (
      !lookupPump(
        arg1,
        address,
        pumpName,
        description
      )
    ) {

      Serial.println(
        F("Unknown pump.")
      );

      Serial.println(
        F("Use: phdn, nuta, nutb, ecdn")
      );

      return;
    }


    const float mL =
      arg2.toFloat();


    if (mL <= 0.0f) {

      Serial.println(
        F("Enter a positive mL value.")
      );

      return;
    }


    Serial.print(F("Commanding "));

    Serial.print(description);

    Serial.print(F(" @ 0x"));

    Serial.print(address, HEX);

    Serial.print(F(": "));

    Serial.print(mL, 2);

    Serial.println(F(" mL"));


    const bool ok =
      pumpDose_mL(
        address,
        mL
      );


    if (ok) {

      Serial.println(
        F("I2C command accepted.")
      );

    } else {

      Serial.println(
        F("FAILED: pump did not acknowledge I2C command.")
      );
    }


    return;
  }


  /* ----------------------------------------------------------
     TIME / PRIME
  ---------------------------------------------------------- */

  if (
    command == "time" ||
    command == "prime"
  ) {

    if (
      arg1.length() == 0 ||
      arg2.length() == 0
    ) {

      Serial.print(F("Usage: "));

      Serial.print(command);

      Serial.println(
        F(" <pump> <ms>")
      );

      Serial.print(F("Example: "));

      Serial.print(command);

      Serial.println(
        F(" nuta 2500")
      );

      return;
    }


    uint8_t address;

    const char* pumpName;
    const char* description;


    if (
      !lookupPump(
        arg1,
        address,
        pumpName,
        description
      )
    ) {

      Serial.println(
        F("Unknown pump.")
      );

      Serial.println(
        F("Use: phdn, nuta, nutb, ecdn")
      );

      return;
    }


    const long enteredTime =
      arg2.toInt();


    if (enteredTime <= 0) {

      Serial.println(
        F("Enter a positive time in milliseconds.")
      );

      return;
    }


    const unsigned long ms =
      (unsigned long)enteredTime;


    Serial.print(F("Commanding "));

    Serial.print(description);

    Serial.print(F(" for "));

    Serial.print(ms);

    Serial.println(F(" ms"));


    const bool ok =
      pumpRun_timeMs(
        address,
        ms
      );


    if (ok) {

      Serial.println(
        F("I2C command accepted.")
      );

    } else {

      Serial.println(
        F("FAILED: pump did not acknowledge I2C command.")
      );
    }


    return;
  }


  /* ----------------------------------------------------------
     UNKNOWN COMMAND
  ---------------------------------------------------------- */

  Serial.println(
    F("Unknown command. Type 'help' for usage.")
  );
}


/* ============================================================
   ARDUINO SETUP
   ============================================================ */

void setup() {

  Serial.begin(
    SERIAL_BAUD
  );


  Wire.begin();


  delay(100);


  Serial.println();
  Serial.println(
    F("====================================================")
  );

  Serial.println(
    F(" Hydroponics Peristaltic Pump Tester READY")
  );

  Serial.println(
    F("====================================================")
  );


  Serial.println();

  Serial.println(
    F("IMPORTANT: For initial identification, use WATER")
  );

  Serial.println(
    F("in all four pump inlet tubes.")
  );


  printHelp();

  printList();
}


/* ============================================================
   ARDUINO MAIN LOOP
   ============================================================ */

void loop() {

  /*
     Collect one complete command from Serial Monitor.

     Commands are processed when a newline character arrives.
  */

  static String buffer;


  while (Serial.available()) {

    const char c =
      (char)Serial.read();


    // Ignore carriage return.
    if (c == '\r') {
      continue;
    }


    // End of command.
    if (c == '\n') {

      parseLine(buffer);

      buffer = "";

      continue;
    }


    /*
       Prevent an accidentally enormous Serial command from
       consuming excessive RAM.
    */

    if (buffer.length() < 100) {

      buffer += c;

    } else {

      buffer = "";

      Serial.println(
        F("ERROR: Serial command too long; buffer cleared.")
      );
    }
  }
}
