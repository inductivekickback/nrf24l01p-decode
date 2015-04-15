# nrf24l01p-decode
A program for decoding transcripts of SPI transactions between a microcontroller and an [nRF24L01+ radio IC](http://www.nordicsemi.com/eng/Products/2.4GHz-RF/nRF24L01P/%28language%29/eng-GB). In addition to making the transcript more readable, the program attempts to decipher information such as the packet format and addresses used. Minimal support for nRF24L01+ clones that are produced by Beken is included.

## About
Many wireless devices use Nordic Semiconductor's ShockBurst and Enhanced ShockBurst 2.4GHz RF protocols. The nRF24L01+ IC (and its knock-offs) are interesting because they are widely-used, compatible with contemporary Nordic radio ICs, and somewhat hacker-friendly.

The nRF24L01+ is an SPI device. To see how it is being used, a logic analyzer is required. [This one](https://www.sparkfun.com/products/13195) can be used to record SPI transactions that look like this:

```
Time [s],Packet ID,MOSI,MISO
0.000002166666667,0,0x07,0x0E
0.000037833333333,0,0x00,0x0E
0.000085000000000,1,0x20,0x0E
0.000120666666667,1,0x0C,0x00
```

The [SPI bus](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus) is bidirectional and always shifts a byte in whenever it shifts a byte out (MISO and MOSI). Packet IDs are used because transactions that consist of consecutive bytes are distributed over multiple lines.

In order to make the transcript easier to read, this program translates the lines above into the following form:

```
0000:R_REGISTER(STATUS):      (RX_P_NO_2|RX_P_NO_1|RX_P_NO_0)
0001:W_REGISTER(CONFIG):      (EN_CRC|CRC0)
```

Each line of the translated file contains the transaction's Packet ID followed by the name of the command, the register being operated on (when appropriate), and the data that was transferred (if any). Additionally, the time delta between consecutive RX payload reads and TX payload writes are included in the translated file. In the following snippet, the nRF24L01+ is being commanded to send 10 bytes of data on channel 0x16 and then another 10 bytes on channel 0x36 4 milliseconds later:

```
4988:W_REGISTER(RF_CH):       0x16
4989:FLUSH_TX:                0x00
4990:W_TX_PAYLOAD:            {0x00,0x00,0x00,0x00,0x00,0x45,0x00,0x00,0x00,0x9A}
4991:R_REGISTER(STATUS):      (TX_DS|RX_P_NO_2|RX_P_NO_1|RX_P_NO_0)
4992:W_REGISTER(STATUS):      (TX_DS|RX_P_NO_2|RX_P_NO_1|RX_P_NO_0)
4993:W_REGISTER(RF_CH):       0x36
4994:FLUSH_TX:                0x00
4995:W_TX_PAYLOAD(delta:0.0040s):{0x00,0x00,0x00,0x00,0x00,0x45,0x00,0x00,0x00,0x9A}
4996:R_REGISTER(STATUS):      (TX_DS|RX_P_NO_2|RX_P_NO_1|RX_P_NO_0)
```

Finally, the program is able to generate initialization code that can be used to configure [Nordic Semiconductor's micro-esb](https://github.com/NordicSemiconductor/nrf51-micro-esb) library on nRF51 devices.

## Usage
To create a translated output file:

```
$ python nrf24l01p-decode.py -i INPUT_FILE_PATH -o OUTPUT_FILE_PATH
```

To create a file containing micro-esb configuration code:

```
$ python nrf24l01p-decode.py -i INPUT_FILE_PATH -u UESB_FILE_PATH
```

The `-o` and `-u` options can also be combined.

Sample [input](docs/SAMPLE_INPUT.txt), [output](docs/SAMPLE_OUTPUT.txt), and [micro-esb configuration](docs/SAMPLE_UESB_CONFIG.txt) files can be found in the docs folder.
