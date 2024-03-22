blackstar_ecm is a C++ project which:
    - Uses the Embedded Processing Unit (EPU) mini-PCIe CAN module to communicate with the MCM.
    - Uses the VersaLogic API module to:
        - Control front panel LEDs
        - Control PSU/Fan Controller I2C
        - Control RF Power Monitor SPI

CAN messages:
    - A minimal message set is supported to allow the BSM (BlackStar Module) to emulate an ECM.
    - Fixed message responses are used where appropriate to provide a minimal ECM emulator.
