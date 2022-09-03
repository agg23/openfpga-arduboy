# Arduboy for Analogue Pocket

A highly modified port of https://github.com/MiSTer-devel/Arduboy_MiSTer by uXeBoy. This core has been updated to the latest Atmega core created by [Iulian Gheorghiu](https://github.com/dev-board-tech).

Unlike the MiSTer port, this core can directly use `hex` roms that are typically provided for use on [Arduboy](https://www.arduboy.com/).

## Limitations

The core currently does not have EEPROM or save state support, so there is no persistence. This will be coming in a future update.

Core may exibit strange audio behavior on a few games. The cause of this issue is unknown at this time. Settings are provided to tweak the audio output to make it more agreeable.

As far as we can tell, any tearing shown by this core is also presented on device. If you can prove this is not the case, please let me know.

## Settings

The Arduboy has a unique sound production system, in that it uses a piezoelectric buzzer to produce sound. This buzzer is wired between two pins (_not_ to ground), so games have some additional flexibility in how they produce sounds.

| Setting         | Action                                                                                                              |
|-----------------|---------------------------------------------------------------------------------------------------------------------|
| Load ROM        | Opens the file browser to select a new hex file, and restarts the core                                              |
| Enable Buzzer 1 | Enables the use of buzzer pin 1 (Recommended)                                                                       |
| Enable Buzzer 2 | Enables the use of buzzer pin 2 (Recommended). If you are experiencing strange sounds, you may want to disable this |
| Limit Volume    | Decreases the output volume (Recommended). The Arduboy doesn't use a speaker, so it sounds quite loud using one     |