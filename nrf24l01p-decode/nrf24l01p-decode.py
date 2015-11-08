#!/usr/bin/env python
"""This program takes a transcript of SPI traffic between a microcontroller
and an nRF24L01+ as input and outputs the transcript in a form that is easier
for a human to read. It can also output configuration code that makes it easier
to replace the nRF24L01+ with an nRF51 that is using the micro-esb
library (nrf51-micro-esb).

For example, the following input lines:

    0.000002166666667,0,0x07,0x0E
    0.000037833333333,0,0x00,0x0E

would be translated to this:

    0000:R_REGISTER(STATUS):      (RX_P_NO_2|RX_P_NO_1|RX_P_NO_0)

Additionally, a time delta is printed that shows the spacing between consecutive
TX payload writes as well as consecutive RX payload reads. This delta can be
used to determine the underlying protocol's timing.

    5000:W_TX_PAYLOAD(delta:0.0040s):{0x00,0x00,0x45,0x00,0x00,0x00,0x9A}

Finally, the output file contains a summary of some important characteristics
that have been gleaned from the input file. Here is an example summary:

    Packet format:            SB
    Data rate:                250KBPS
    CRC width:                16BIT
    Address width:            5
    Possible channels:        [2, 8, 75, 48, 64, 9, 54, 30, 62, 22]
    Output power:             0dBm
    Auto retransmit count:    15
    Auto retransmit delay:    3750
    Packets sent:             991
    Packets received:         0

"""
import argparse
import sys
import datetime
import os
import os.path

VERSION = (0.1, (14, 4, 2015))

COL_SEPARATOR = ','
EXPECTED_COL_NAMES = ('Time [s]', 'Packet ID', 'MOSI', 'MISO')


class DecodeError(Exception):
    """Subclass for reporting errors."""
    pass


class Decode(object):
    """A simple class for parsing nRF24L01+ SPI traffic."""

    # Each register has an addr, description, initial value,
    # and a mask of its writable bits.
    REGISTERS = {
        0x00: ('CONFIG',      (0x08,), 0x7F),
        0x01: ('EN_AA',       (0x3F,), 0x3F),
        0x02: ('EN_RXADDR',   (0x03,), 0x3F),
        0x03: ('SETUP_AW',    (0x03,), 0x03),
        0x04: ('SETUP_RETR',  (0x03,), 0xFF),
        0x05: ('RF_CH',       (0x02,), 0x7F),
        0x06: ('RF_SETUP',    (0x0E,), 0xBF),
        0x07: ('STATUS',      (0x0E,), 0x70),
        0x08: ('OBSERVE_TX',  (0x00,), 0x00),
        0x09: ('RPD',         (0x00,), 0x00),
        0x0A: ('RX_ADDR_P0',  (0xE7, 0xE7, 0xE7, 0xE7, 0xE7), 0xFF),
        0x0B: ('RX_ADDR_P1',  (0xC2, 0xC2, 0xC2, 0xC2, 0xC2), 0xFF),
        0x0C: ('RX_ADDR_P2',  (0xC3,), 0xFF),
        0x0D: ('RX_ADDR_P3',  (0xC4,), 0xFF),
        0x0E: ('RX_ADDR_P4',  (0xC5,), 0xFF),
        0x0F: ('RX_ADDR_P5',  (0xC6,), 0xFF),
        0x10: ('TX_ADDR',     (0xE7, 0xE7, 0xE7, 0xE7, 0xE7), 0xFF),
        0x11: ('RX_PW_P0',    (0x00,), 0x3F),
        0x12: ('RX_PW_P1',    (0x00,), 0x3F),
        0x13: ('RX_PW_P2',    (0x00,), 0x3F),
        0x14: ('RX_PW_P3',    (0x00,), 0x3F),
        0x15: ('RX_PW_P4',    (0x00,), 0x3F),
        0x16: ('RX_PW_P5',    (0x00,), 0x3F),
        0x17: ('FIFO_STATUS', (0x11,), 0x00),
        0x1C: ('DYNPD',       (0x00,), 0x3F),
        0x1D: ('FEATURE',     (0x00,), 0x07)
    }

    # The individual names of the register bits.
    REGISTER_FIELDS = {
        0x00: ('PRIM_RX',
               'PWR_UP',
               'CRC0',
               'EN_CRC',
               'MASK_MAX_RT',
               'MASK_TX_DS',
               'MASK_RX_DR',
               None),
        0x01: ('ENAA_P0',
               'ENAA_P1',
               'ENAA_P2',
               'ENAA_P3',
               'ENAA_P4',
               'ENAA_P5',
               None,
               None),
        0x02: ('ERX_P0',
               'ERX_P1',
               'ERX_P2',
               'ERX_P3',
               'ERX_P4',
               'ERX_P5',
               None,
               None),
        0x03: ('AW_0',
               'AW_1',
               None,
               None,
               None,
               None,
               None,
               None),
        0x04: ('ARC_0',
               'ARC_1',
               'ARC_2',
               'ARC_3',
               'ARD_0',
               'ARD_1',
               'ARD_2',
               'ARD_3'),
        0x06: (None,
               'RF_PWR_0',
               'RF_PWR_1',
               'RF_DR_HIGH',
               'PLL_LOCK',
               'RF_DR_LOW',
               None,
               'CONT_WAVE'),
        0x07: ('TX_FULL',
               'RX_P_NO_0',
               'RX_P_NO_1',
               'RX_P_NO_2',
               'MAX_RT',
               'TX_DS',
               'RX_DR',
               None),
        0x08: ('ARC_CNT_0',
               'ARC_CNT_1',
               'ARC_CNT_2',
               'ARC_CNT_3',
               'PLOS_CNT_0',
               'PLOS_CNT_1',
               'PLOS_CNT_2',
               'PLOS_CNT_3'),
        0x09: ('CD',
               None,
               None,
               None,
               None,
               None,
               None,
               None),
        0x17: ('RX_EMPTY',
               'RX_FULL',
               None,
               None,
               'TX_EMPTY',
               'TX_FULL',
               'TX_REUSE',
               None),
        0x1C: ('DPL_P0',
               'DPL_P1',
               'DPL_P2',
               'DPL_P3',
               'DPL_P4',
               'DPL_P5',
               None,
               None),
        0x1D: ('EN_DYN_ACK',
               'EN_ACK_PAY',
               'EN_DPL',
               None,
               None,
               None,
               None,
               None)
    }

    # These two registers contain an index as part of the command byte itself.
    RW_REGISTER_CMD_MASK = 0xE0
    RW_REGISTER_VALUE_MASK = 0x1F
    RW_REGISTER_MAX_INDEX = 0x1D

    W_ACK_PAYLOAD_VALUE_MASK = 0x07
    W_ACK_PAYLOAD_CMD_MASK = 0xF8
    W_ACK_PAYLOAD_MAX_INDEX = 7

    # Each command has a value, description, and (min, max) data length.
    COMMANDS = {
        0x00: ('R_REGISTER',          (1, 5)), # Bottom 5 bits is register index.
        0x20: ('W_REGISTER',          (1, 5)), # Bottom 5 bits is register index.
        0x61: ('R_RX_PAYLOAD',        (1, 32)),
        0xA0: ('W_TX_PAYLOAD',        (1, 32)),
        0xB0: ('W_TX_PAYLOAD_NO_ACK', (1, 32)),
        0xE1: ('FLUSH_TX',            (0, 1)), # Some devices add a byte to this
                                               # command for some reason.
        0xE2: ('FLUSH_RX',            (0, 0)),
        0xE3: ('REUSE_TX_PL',         (0, 0)),
        0x50: ('ACTIVATE',            (1, 1)),
        0x60: ('R_RX_PL_WID',         (1, 1)),
        0xA8: ('W_ACK_PAYLOAD',       (1, 32)), # Bottom 3 bits is pipe index.
        0xFF: ('NOP',                 (0, 0))
    }

    REGISTER_LOOKUP = {
        'CONFIG':      0x00,
        'EN_AA':       0x01,
        'EN_RXADDR':   0x02,
        'SETUP_AW':    0x03,
        'SETUP_RETR':  0x04,
        'RF_CH':       0x05,
        'RF_SETUP':    0x06,
        'STATUS':      0x07,
        'OBSERVE_TX':  0x08,
        'CD':          0x09,
        'RX_ADDR_P0':  0x0A,
        'RX_ADDR_P1':  0x0B,
        'RX_ADDR_P2':  0x0C,
        'RX_ADDR_P3':  0x0D,
        'RX_ADDR_P4':  0x0E,
        'RX_ADDR_P5':  0x0F,
        'TX_ADDR':     0x10,
        'RX_PW_P0':    0x11,
        'RX_PW_P1':    0x12,
        'RX_PW_P2':    0x13,
        'RX_PW_P3':    0x14,
        'RX_PW_P4':    0x15,
        'RX_PW_P5':    0x16,
        'FIFO_STATUS': 0x17,
        'DYNPD':       0x1C,
        'FEATURE':     0x1D
    }

    REGISTER_FIELD_LOOKUP = {
        'PRIM_RX':             0,
        'PWR_UP':              1,
        'CRC0':                2,
        'EN_CRC':              3,
        'MASK_MAX_RT':         4,
        'MASK_TX_DS':          5,
        'MASK_RX_DR':          6,
        'ENAA_P0':             0,
        'ENAA_P1':             1,
        'ENAA_P2':             2,
        'ENAA_P3':             3,
        'ENAA_P4':             4,
        'ENAA_P5':             5,
        'ERX_P0':              0,
        'ERX_P1':              1,
        'ERX_P2':              2,
        'ERX_P3':              3,
        'ERX_P4':              4,
        'ERX_P5':              5,
        'AW_0':                0,
        'AW_1':                1,
        'ARC_0':               0,
        'ARC_1':               1,
        'ARC_2':               2,
        'ARC_3':               3,
        'ARD_0':               4,
        'ARD_1':               5,
        'ARD_2':               6,
        'ARD_3':               7,
        'LNA_HCURR':           0,
        'RF_PWR_0':            1,
        'RF_PWR_1':            2,
        'RF_DR_HIGH':          3,
        'PLL_LOCK':            4,
        'RF_DR_LOW':           5,
        'CONT_WAVE':           7,
        'STATUS.TX_FULL':      0,
        'RX_P_NO_0':           1,
        'RX_P_NO_1':           2,
        'RX_P_NO_2':           3,
        'MAX_RT':              4,
        'TX_DS':               5,
        'RX_DR':               6,
        'ARC_CNT_0':           0,
        'ARC_CNT_1':           1,
        'ARC_CNT_2':           2,
        'ARC_CNT_3':           3,
        'PLOS_CNT_0':          4,
        'PLOS_CNT_1':          5,
        'PLOS_CNT_2':          6,
        'PLOS_CNT_3':          7,
        'CD':                  0,
        'RX_EMPTY':            0,
        'RX_FULL':             1,
        'TX_EMPTY':            4,
        'FIFO_STATUS.TX_FULL': 5,
        'TX_REUSE':            6,
        'DPL_P0':              0,
        'DPL_P1':              1,
        'DPL_P2':              2,
        'DPL_P3':              3,
        'DPL_P4':              4,
        'DPL_P5':              5,
        'EN_DYN_ACK':          0,
        'EN_ACK_PAY':          1,
        'EN_DPL':              2
    }

    # These STATUS fields are cleared by writing to them.
    XOR_FIELDS = ('RX_DR', 'TX_DS', 'MAX_RT')
    XOR_BIT_MASKS = tuple([(1 << REGISTER_FIELD_LOOKUP[field]) for field in XOR_FIELDS])

    BEKEN_BANK_SWITCH_DATA = 0x53

    # These are the Auto Retry Count fields in SETUP_RETR.
    ARC_MASK = 0x0F

    # These are the Address Width fields in SETUP_AW.
    AW_MASK = 0x03

    RF_CHANNEL_MASK = 0x7F

    OUTPUT_POWER_MASK = 0x06
    OUTPUT_POWER_OFFSET = 1

    ARD_MASK = 0xF0
    ARD_OFFSET = 4
    ARD_MULTIPLIER_US = 250

    PIPE_CONFIG_REGISTERS = (
        'TX_ADDR',
        'EN_RXADDR',
        'RX_ADDR_P0',
        'RX_ADDR_P1',
        'RX_ADDR_P2',
        'RX_ADDR_P3',
        'RX_ADDR_P4',
        'RX_ADDR_P5',
        'RX_PW_P0',
        'RX_PW_P1',
        'RX_PW_P2',
        'RX_PW_P3',
        'RX_PW_P4',
        'RX_PW_P5',
        'DYNPD'
    )

    RX_PW_REGISTERS = ['RX_PW_P0',
                       'RX_PW_P1',
                       'RX_PW_P2',
                       'RX_PW_P3',
                       'RX_PW_P4',
                       'RX_PW_P5']
    RX_PW_MASK = 0x3F

    # It is assumed that the radio will always be in one of these modes.
    OPERATIONAL_MODE = ('POWER_DOWN', 'STANDBY', 'TX_MODE', 'RX_MODE')

    PACKET_FORMAT = ('ESB', 'SB', 'ESB_DPL')

    def __init__(self, **kwargs):
        """Creates a new object. Does not expect any arguments."""
        self.reg_values = {}
        self.messages = []
        self.used_channels = []

        self.tx_count = 0
        self.rx_count = 0

        self._timestamps = {}

        # Beken devices are prevalent and have subtle
        # operational differences.
        self._beken_bank_switch_active = False
        self.beken_detected = False

        self.reset()

    def reset(self):
        """Resets the internal state of the object. The internal state is
        updated whenever SPI traffic is parsed.

        """
        default_rf_ch = self.REGISTERS[self.REGISTER_LOOKUP['RF_CH']][1][0]

        self.reg_values = {}
        self.messages = []
        self.used_channels = [default_rf_ch]

        self.tx_count = 0
        self.rx_count = 0

        self._timestamps = {}

        self._beken_bank_switch_active = False
        self.beken_detected = False

        for addr, props in self.REGISTERS.iteritems():
            desc, init_value, writable_mask = props
            self.reg_values[addr] = list(init_value)

    def update(self, ts, transaction_id, mosi_data, miso_data):
        """Updates the internal state of the object. Expects the following params:
            ts                [float]            Timestamp of transaction in seconds
            transaction_id    [int]              Transaction ID
            mosi_data         [tuple of ints]    MOSI bytes
            miso_data         [tuple of ints]    MISO bytes

        """
        cmd = mosi_data[0]
        mosi_data = mosi_data[1:]

        status = miso_data[0]
        miso_data = miso_data[1:]

        if (len(mosi_data) != len(miso_data)):
            sys.stderr.write('ERROR: MISO and MOSI data lengths do not match\r\n')
            return

        cmd_props = self.COMMANDS.get(cmd & self.RW_REGISTER_CMD_MASK)
        if (cmd_props is not None):
            packed_index = (cmd & self.RW_REGISTER_VALUE_MASK)
            self._update(ts,
                         transaction_id,
                         cmd_props,
                         status,
                         mosi_data,
                         miso_data,
                         packed_index)

        cmd_props = self.COMMANDS.get(cmd & self.W_ACK_PAYLOAD_CMD_MASK)
        if (cmd_props is not None):
            packed_index = (cmd & self.W_ACK_PAYLOAD_VALUE_MASK)
            self._update(ts,
                         transaction_id,
                         cmd_props,
                         status,
                         mosi_data,
                         miso_data,
                         packed_index)

        cmd_props = self.COMMANDS.get(cmd)
        if (cmd_props is not None):
            self._update(ts,
                         transaction_id,
                         cmd_props,
                         status,
                         mosi_data,
                         miso_data)

        raise DecodeError('ERROR: Failed to process command: 0x%X' % cmd)

    def get_data_rate(self):
        """Returns one of the following strs: '250KBPS', '1MBPS', '2MBPS'."""
        if (1 == self._read_state_reg('RF_SETUP', 'RF_DR_LOW')):
            return '250KBPS'
        elif (0 == self._read_state_reg('RF_SETUP', 'RF_DR_HIGH')):
            return '1MBPS'
        else:
            return '2MBPS'

    def get_channel(self):
        """Returns the current RF channel as an int."""
        return (self._read_state_reg('RF_CH')[0] & self.RF_CHANNEL_MASK)

    def get_operational_mode(self):
        """Returns one of the following strs: 'POWER_DOWN', 'STANDBY', 'PRX', 'PTX'.

        NOTE: The two different standby modes are indistinguishable without CE
        visibility.

        """
        if (0 == self._read_state_reg('CONFIG', 'PWR_UP')):
            return 'POWER_DOWN'

        if (0 == self._read_state_reg('CONFIG', 'PRIM_RX')):
            if (1 == self._read_state_reg('FIFO_STATUS', 'TX_EMPTY')):
                return 'STANDBY'
            else:
                return 'PTX'
        else:
            return 'PRX'

        return 'STANDBY'

    def get_packet_format(self):
        """Returns one of the following strs: 'SB', 'ESB', 'ESB_DPL'."""
        dpl = self._read_state_reg('FEATURE', 'EN_DPL')

        # According to Beken app note BK2423 v2
        if (self.beken_detected):
            if ((0 == self._read_state_reg('EN_AA')[0]) and (0 == dpl)):
                return 'SB'
            elif (1 == dpl):
                return 'ESB_DPL'
            else:
                return 'ESB'

        # ShockBurst packets are sent if EN_AA=0x00, ARC=0, and baudrate is 1Mbps.
        if (0 == self._read_state_reg('EN_AA')[0]):
            if (0 == (self._read_state_reg('SETUP_RETR')[0] & self.ARC_MASK)):
                dr = self.get_data_rate()
                if (('1MBPS' == dr) or ('250KBPS' == dr)):
                    return 'SB'

        if (1 == self._read_state_reg('FEATURE', 'EN_DPL')):
            return 'ESB_DPL'
        else:
            return 'ESB'

    def get_CRC_mode(self):
        """Returns one of the following strs: 'OFF', '8BIT', '16BIT'."""
        if ('SB' == self.get_packet_format()):
            if (0 == self._read_state_reg('CONFIG', 'EN_CRC')):
                return 'OFF'

        if (0 == self._read_state_reg('CONFIG', 'CRC0')):
            return '8BIT'
        else:
            return '16BIT'

    def get_address_width(self):
        """Returns one of the following address widths (in bytes): 3, 4, 5."""
        val = (self._read_state_reg('SETUP_AW')[0] & self.AW_MASK)
        if (0x01 == val):
            return 3
        elif (0x02 == val):
            return 4
        elif (0x03 == val):
            return 5
        else:
            return 0

    def get_used_channels(self):
        """Returns a list of RF channels that may have been used (as ints)."""
        return self.used_channels

    def get_pipe_config(self):
        """Returns a dict with the keys from PIPE_CONFIG_REGISTERS along with
        their values.

        """
        result = {}
        for reg in self.PIPE_CONFIG_REGISTERS:
            val = self._read_state_reg(reg)
            if (1 == len(val)):
                result[reg] = val[0]
            else:
                result[reg] = val
        return result

    def get_output_power(self):
        """Returns one of the following strs: '-18dBm', '-12dBm', '-6dBm', '0dBm'.
        NOTE: These levels are specific to the nRF24L01+ and may not be accurate
        for Beken devices.

        """
        val = (self._read_state_reg('RF_SETUP')[0] & self.OUTPUT_POWER_MASK)
        val = (val >> self.OUTPUT_POWER_OFFSET)
        if (0x00 == val):
            return '-18dBm'
        elif (0x01 == val):
            return '-12dBm'
        elif (0x10 == val):
            return '-6dBm'
        else:
            return '0dBm'

    def get_auto_retransmit_count(self):
        """Returns the Auto Retransmit Count as an int. When using an
        nRF24L01+, a non-zero value here implies the usage of the
        Enhanced ShockBurst protocol.

        """
        return (self._read_state_reg('SETUP_RETR')[0] & self.ARC_MASK)

    def get_auto_retransmit_delay(self):
        """Returns the Auto Retransmit Delay as an int (in microseconds)."""
        val = ((self._read_state_reg('SETUP_RETR')[0] & self.ARD_MASK) >> self.ARD_OFFSET)
        return (self.ARD_MULTIPLIER_US * val)

    def get_tx_count(self):
        """Returns the number of W_TX_PAYLOAD and W_TX_PAYLOAD_NO_ACK commands
        that were found in the input file.

        """
        return self.tx_count

    def get_rx_count(self):
        """Returns the number of R_RX_PAYLOAD commands that were found in the
        input file.

        """
        return self.rx_count

    def get_uesb_config(self):
        """Returns a str containing code that can be used to initialize the
        micro-esb library on the nRF51.

        """
        result = []
        pc = self.get_pipe_config()
        op = self.get_operational_mode()

        if (self.beken_detected):
            result.append('// NOTE: The device appears to be a Nordic clone (e.g. Beken BK2423).')

        # The micro-esb library uses PIPE0 for TX.
        if ('PRX' != op):
            result.append('{:<43s}= {:s}; {:s}'.format('const uint8_t rx_addr_p0[]',
                                                       self._format_num(pc['TX_ADDR']),
                                                       '// Using TX_ADDR because mode is PTX.'))
        else:
            result.append('{:<43s}= {:s};'.format('const uint8_t rx_addr_p0[]',
                                                  self._format_num(pc['RX_ADDR_P0'])))

        result.append('{:<43s}= {:s};'.format('const uint8_t rx_addr_p1[]',
                                              self._format_num(pc['RX_ADDR_P1'])))
        result.append('{:<43s}'.format('uint32_t      uesb_err;'))
        result.append('')
        result.append('{:<43s}= {:s};'.format('uesb_config_t uesb_config',
                                              'UESB_DEFAULT_CONFIG'))
        result.append('{:<43s}= {:d};'.format('uesb_config.rf_channel',
                                              self.get_channel()))
        result.append('{:<43s}= {:s};'.format('uesb_config.crc',
                                              ('UESB_CRC_' + self.get_CRC_mode())))

        pws = [(self._read_state_reg(reg)[0] & self.RX_PW_MASK) for reg in self.RX_PW_REGISTERS]
        min_pw = min(pws)
        max_pw = max(pws)
        if ((min_pw != max_pw) and ('ESB_DPL' != self.get_packet_format())):
            result.append('// ERROR: The RX_PW_PX pipes have different ' +
                          'configurations and the mode is not ESB_DPL.')
        else:
            result.append('{:<43s}= {:d};'.format('uesb_config.payload_length', max_pw))

        result.append('{:<43s}= {:s};'.format('uesb_config.protocol',
                                              ('UESB_PROTOCOL_' + self.get_packet_format())))
        result.append('{:<43s}= {:s};'.format('uesb_config.bitrate',
                                              ('UESB_BITRATE_' + self.get_data_rate())))

        if ('PRX' == op):
            result.append('{:<43s}= {:s};'.format('uesb_config.mode',
                                                  'UESB_MODE_PRX'))
        else:
            result.append('{:<43s}= {:s};'.format('uesb_config.mode',
                                                  'UESB_MODE_PTX'))

        result.append('{:<43s}= {:d};'.format('uesb_config.rf_addr_length',
                                              self.get_address_width()))

        # The power levels don't match up exactly so we'll translate.
        pwr = self.get_output_power()
        if ('0dBm' == pwr):
            result.append('{:<43s}= {:s};'.format('uesb_config.tx_output_power',
                                                  'UESB_TX_POWER_0DBM'))
        elif ('-6dBm' == pwr):
            result.append('{:<43s}= {:s};'.format('uesb_config.tx_output_power',
                                                  'UESB_TX_POWER_NEG4DBM'))
        elif ('-12dBm' == pwr):
            result.append('{:<43s}= {:s};'.format('uesb_config.tx_output_power',
                                                  'UESB_TX_POWER_NEG12DBM'))
        elif ('-18dBm' == pwr):
            result.append('{:<43s}= {:s};'.format('uesb_config.tx_output_power',
                                                  'UESB_TX_POWER_NEG16DBM'))

        result.append('{:<43s}= {:s};'.format('uesb_config.rx_address_p2',
                                              self._format_num(pc['RX_ADDR_P2'])))
        result.append('{:<43s}= {:s};'.format('uesb_config.rx_address_p3',
                                              self._format_num(pc['RX_ADDR_P3'])))
        result.append('{:<43s}= {:s};'.format('uesb_config.rx_address_p4',
                                              self._format_num(pc['RX_ADDR_P4'])))
        result.append('{:<43s}= {:s};'.format('uesb_config.rx_address_p5',
                                              self._format_num(pc['RX_ADDR_P5'])))

        if ((not self.beken_detected) or (0 == self._read_state_reg('FEATURE', 'EN_DPL'))):
            if (1 == self._read_state_reg('FEATURE', 'EN_DYN_ACK')):
                result.append('{:<43s}= {:d};'.format('uesb_config.dynamic_ack_enabled', 1))
            else:
                result.append('{:<43s}= {:d};'.format('uesb_config.dynamic_ack_enabled', 0))
        else:
            result.append('{:<43s}= {:d}; {:s}'.format('uesb_config.dynamic_ack_enabled',
                                                       1,
                                                       '// NOTE: According to Beken app note BK2423 v2'))

        if (0 == pc['DYNPD']):
            result.append('uesb_config.dynamic_payload_length_enabled = 0;' +
                          ' // Used in PRX mode')
        else:
            result.append('uesb_config.dynamic_payload_length_enabled = 1;' +
                          ' // Used in PRX mode')

        result.append('{:<43s}= {:s};'.format('uesb_config.rx_pipes_enabled',
                                              self._format_num(pc['EN_RXADDR'])))
        result.append('{:<43s}= {:d};'.format('uesb_config.retransmit_delay',
                                              self.get_auto_retransmit_delay()))
        result.append('{:<43s}= {:d};'.format('uesb_config.retransmit_count',
                                              self.get_auto_retransmit_count()))
        result.append('{:<43s}= {:s}; {:s}'.format('uesb_config.event_handler',
                                                   '0',
                                                   '// TODO: Set event handler'))
        result.append('')
        result.append('uesb_err = uesb_init(&uesb_config);')
        result.append('if (UESB_SUCCESS != uesb_err)')
        result.append('{')
        result.append('    // TODO: Handle the error.')
        result.append('}')
        result.append('')
        result.append('uesb_err = uesb_set_address(UESB_ADDRESS_PIPE0, &rx_addr_p0[0]);')
        result.append('if (UESB_SUCCESS != uesb_err)')
        result.append('{')
        result.append('    // TODO: Handle the error.')
        result.append('}')
        result.append('')
        result.append('uesb_err = uesb_set_address(UESB_ADDRESS_PIPE1, &rx_addr_p1[0]);')
        result.append('if (UESB_SUCCESS != uesb_err)')
        result.append('{')
        result.append('    // TODO: Handle the error.')
        result.append('}')
        result.append('')

        return os.linesep.join(result)

    def _bit_is_set(self, val, bit):
        if (isinstance(bit, str)):
            bit = self.REGISTER_FIELD_LOOKUP[bit]
        return ((val >> bit) & 1)

    def _set_bit(self, val, bit):
        if (isinstance(bit, str)):
            bit = self.REGISTER_FIELD_LOOKUP[bit]
        return (val | (1 << bit))

    def _clear_bit(self, val, bit):
        if (isinstance(bit, str)):
            bit = self.REGISTER_FIELD_LOOKUP[bit]
        return (val & ~(1 << bit))

    def _read_state_reg(self, reg, bit=None):
        if (isinstance(reg, str)):
            reg = self.REGISTER_LOOKUP[reg]
        val = self.reg_values[reg]
        if (bit is None):
            return val
        else:
            if (1 != len(val)):
                raise DecodeError('ERROR: Ambiguous _read_state_reg operation.')
            return self._bit_is_set(val[0], bit)

    def _clear_state_reg_bit(self, reg, bit):
        if (isinstance(reg, str)):
            reg = self.REGISTER_LOOKUP[reg]
        if (isinstance(bit, str)):
            bit = self.REGISTER_FIELD_LOOKUP[bit]
        if (1 != len(self.reg_values[reg])):
            raise DecodeError('ERROR: Ambiguous _clear_state_reg_bit operation')
        self.reg_values[reg][0] &= ~(1 << bit)

    def _set_state_reg_bit(self, reg, bit):
        if (isinstance(reg, str)):
            reg = self.REGISTER_LOOKUP[reg]
        if (isinstance(bit, str)):
            bit = self.REGISTER_FIELD_LOOKUP[bit]
        if (1 != len(self.reg_values[reg])):
            raise DecodeError('ERROR: Ambiguous _clear_state_reg_bit operation')
        self.reg_values[reg][0] |= (1 << bit)

    def _update(self, ts, transaction_id, cmd_props, status, mosi_data, miso_data, packed_index=None):
        cmd_name = cmd_props[0]
        min_data_len, max_data_len = cmd_props[1]

        self.reg_values[self.REGISTER_LOOKUP['STATUS']][0] = status

        try:
            func = getattr(self, ('_' + cmd_name.lower()))
            if (min_data_len <= len(mosi_data) <= max_data_len):
                func(ts, transaction_id, mosi_data, miso_data, packed_index)
            else:
                sys.stderr.write('ERROR: Invalid data len for command ' +
                                 '%s: %d\r\n' % (cmd_name, len(mosi_data)))
        except AttributeError:
            raise DecodeError('ERROR: Failed to lookup func: %s' % cmd_name)

    def _seq_to_hex_str(self, seq):
        s = ','.join(['0x%02X' % x for x in seq])
        if (1 == len(seq)):
            return s
        else:
            return ('[' + s + ']')

    def _reg_fields_str(self, reg, value):
        fields = self.REGISTER_FIELDS.get(reg)
        if (fields is None):
            return '0x{:02X}'.format(value)
        else:
            result = []
            for i in range(7, -1, -1):
                if (((value >> i) & 0x01) != 0):
                    if (fields[i] is not None):
                        result.append(fields[i])
                    else:
                        result.append('R')
            if (0 == len(result)):
                return '0x00'
            else:
                return ('(' + '|'.join(result) + ')')

    def _format_num(self, seq):
        result = []
        if (isinstance(seq, int)):
            return '0x{:02X}'.format(seq)

        for item in seq:
            if (isinstance(item, int)):
                result.append('0x{:02X}'.format(item))
            else:
                result.append(item)

        if (1 == len(result)):
            result = result[0]
        else:
            result = ('{' + ','.join(result) + '}')

        return result

    def _msg(self, transaction_id, msg, seq=None):
        id_str = '{:04d}:'.format(transaction_id)
        if (seq is None):
            self.messages.append(id_str + msg)
        else:
            result = self._format_num(seq)
            self.messages.append(id_str + '{:<25}{}'.format((msg + ':'), result))

    def _r_register(self, ts, transaction_id, mosi_data, miso_data, packed_index):
        if (packed_index > self.RW_REGISTER_MAX_INDEX):
            self._msg(transaction_id,
                      '[ERROR: Invalid index found in R_REGISTER command byte: %d]' % packed_index)
            return

        desc, init_value, mask = self.REGISTERS[packed_index]
        reg_width = len(init_value)

        # Beken has several 'nRF24L01-compatible' devices. They execute an ACTIVATE command
        # with payload BEKEN_BANK_SWITCH_DATA to write to a separate bank of registers.
        # For now, these will be ignored.
        if (self._beken_bank_switch_active):
            self._msg(transaction_id,
                      ('[IGNORED: BEKEN-SPECIFIC COMMAND]R_REGISTER(%s)' % desc),
                      [self._reg_fields_str(packed_index, x) for x in miso_data])
            return

        if (len(mosi_data) != reg_width):
            self._msg(transaction_id,
                      ('[IGNORED: INVALID DATA LEN]W_REGISTER(%s)' % desc),
                      [self._reg_fields_str(packed_index, x) for x in mosi_data])
        else:
            for i, data in enumerate(miso_data):
                self.reg_values[packed_index][i] = data

                self._msg(transaction_id,
                          ('R_REGISTER(%s)' % self.REGISTERS[packed_index][0]),
                          [self._reg_fields_str(packed_index, x) for x in miso_data])

    def _w_register(self, ts, transaction_id, mosi_data, miso_data, packed_index):
        if (packed_index > self.RW_REGISTER_MAX_INDEX):
            self._msg(transaction_id,
                      '[ERROR: Invalid index found in W_REGISTER command byte: %d]' % packed_index)
            return

        desc, init_value, mask = self.REGISTERS[packed_index]
        reg_width = len(init_value)

        # Beken has several 'nRF24L01-compatible' devices. They execute an ACTIVATE command
        # with payload BEKEN_BANK_SWITCH_DATA to write to a separate bank of registers.
        # For now, these will be ignored.
        if (self._beken_bank_switch_active):
            self._msg(transaction_id,
                      ('[IGNORED: BEKEN-SPECIFIC COMMAND]W_REGISTER(%s)' % desc),
                      [self._reg_fields_str(packed_index, x) for x in mosi_data])

        # The W_REGISTER command is only executed in 'POWER_DOWN' or 'STANDBY' modes.
        op = self.get_operational_mode()
        if (('POWER_DOWN' == op) or ('STANDBY' == op)):
            if (len(mosi_data) != reg_width):
                self._msg(transaction_id,
                          ('[IGNORED: INVALID DATA LEN]W_REGISTER(%s)' % desc),
                          [self._reg_fields_str(packed_index, x) for x in mosi_data])
                return
            else:
                for i, data in enumerate(mosi_data):
                    val = self.reg_values[packed_index][i]
                    if (packed_index == self.REGISTER_LOOKUP['STATUS']):
                        # The STATUS register is written to clear certain flags.
                        for xor_mask in self.XOR_BIT_MASKS:
                            if (data & xor_mask):
                                val &= ~xor_mask
                    else:
                        val = (val & ~mask)
                        val |= (data & mask)

                    self.reg_values[packed_index][i] = val

                # Some register writes are more interesting than others.
                if (packed_index == self.REGISTER_LOOKUP['RF_CH']):
                    ch = mosi_data[-1]
                    if (not ch in self.used_channels):
                        self.used_channels.append(ch)

                self._msg(transaction_id,
                          ('W_REGISTER(%s)' % desc),
                          [self._reg_fields_str(packed_index, x) for x in mosi_data])
        else:
            self._msg(transaction_id,
                      ('[IGNORED: INVALID OPERATIONAL MODE]W_REGISTER(%s)' % desc),
                      [self._reg_fields_str(packed_index, x) for x in mosi_data])

    def _r_rx_payload(self, ts, transaction_id, mosi_data, miso_data, packed_index):
        self.rx_count += 1

        delta = None
        if (self._timestamps.has_key('R_RX_PAYLOAD')):
            delta = (ts - self._timestamps['R_RX_PAYLOAD'])
        self._timestamps['R_RX_PAYLOAD'] = ts

        if (delta is not None):
            self._msg(transaction_id,
                      ('R_RX_PAYLOAD(delta:%.4fs)' % delta),
                      miso_data)
        else:
            self._msg(transaction_id, 'R_RX_PAYLOAD', miso_data)

    def _w_tx_payload(self, ts, transaction_id, mosi_data, miso_data, packed_index):
        self._clear_state_reg_bit('FIFO_STATUS', 'TX_REUSE')
        self.tx_count += 1

        delta = None
        if (self._timestamps.has_key('W_TX_PAYLOAD')):
            delta = (ts - self._timestamps['W_TX_PAYLOAD'])
        self._timestamps['W_TX_PAYLOAD'] = ts

        if (delta is not None):
            self._msg(transaction_id,
                      ('W_TX_PAYLOAD(delta:%.4fs)' % delta),
                      mosi_data)
        else:
            self._msg(transaction_id, 'W_TX_PAYLOAD', mosi_data)

    def _w_tx_payload_no_ack(self, ts, transaction_id, mosi_data, miso_data, packed_index):
        self.tx_count += 1

        # Both W_TX_PAYLOAD commands are considered equivalent.
        delta = None
        if (self._timestamps.has_key('W_TX_PAYLOAD')):
            delta = (ts - self._timestamps['W_TX_PAYLOAD'])
        self._timestamps['W_TX_PAYLOAD'] = ts

        if (delta is not None):
            self._msg(transaction_id,
                      ('W_TX_PAYLOAD_NO_ACK(delta:%.4fs)' % delta),
                      mosi_data)
        else:
            self._msg(transaction_id, 'W_TX_PAYLOAD_NO_ACK', mosi_data)

    def _flush_tx(self, ts, transaction_id, mosi_data, miso_data, packed_index):
        self._clear_state_reg_bit('FIFO_STATUS', 'TX_REUSE')
        self._clear_state_reg_bit('FIFO_STATUS', 'FIFO_STATUS.TX_FULL')
        self._clear_state_reg_bit('STATUS', 'STATUS.TX_FULL')
        if (0 == len(mosi_data)):
            self._msg(transaction_id, 'FLUSH_TX')
        else:
            self._msg(transaction_id, 'FLUSH_TX', mosi_data)

    def _flush_rx(self, ts, transaction_id, mosi_data, miso_data, packed_index):
        self._clear_state_reg_bit('FIFO_STATUS', 'FIFO_STATUS.RX_FULL')
        self._clear_state_reg_bit('STATUS', 'STATUS.RX_FULL')
        if (0 == len(mosi_data)):
            self._msg(transaction_id, 'FLUSH_RX')
        else:
            self._msg(transaction_id, 'FLUSH_RX', mosi_data)

    def _reuse_tx_pl(self, ts, transaction_id, mosi_data, miso_data, packed_index):
        self._set_state_reg_bit('FIFO_STATUS', 'TX_REUSE')
        self._msg.append(transaction_id, 'REUSE_TX_PL')

    def _activate(self, ts, transaction_id, mosi_data, miso_data, packed_index):
        if (self.BEKEN_BANK_SWITCH_DATA == mosi_data[0]):
            self._beken_bank_switch_active = (not self._beken_bank_switch_active)

        self.beken_detected = True

        self._msg(transaction_id, '[IGNORED: BEKEN-SPECIFIC COMMAND]ACTIVATE', mosi_data)

    def _r_rx_pl_wid(self, ts, transaction_id, mosi_data, miso_data, packed_index):
        self._msg(transaction_id, 'R_RX_PL_WID', miso_data)

    def _w_ack_payload(self, ts, transaction_id, mosi_data, miso_data, packed_index):
        self._msg(transaction_id, 'W_ACK_PAYLOAD', mosi_data)

    def _nop(self, ts, transaction_id, mosi_data, miso_data, packed_index):
        self._msg(transaction_id, 'NOP')

    def __repr__(self):
        return os.linesep.join(self.messages)


def _verify_column_names(line):
    names = [s.strip() for s in line.split(COL_SEPARATOR)]

    if (len(EXPECTED_COL_NAMES) != len(names)):
        raise DecodeError('ERROR: Unexpected number of columns: %d' % len(names))

    for i, s in enumerate(names):
        if (not EXPECTED_COL_NAMES[i] in s):
            raise DecodeError('ERROR: Unexpected column name: expected ' +
                              '%s but found %s' % (names[i], s))


def _parse_num(s):
    try:
        return int(s)
    except ValueError:
        try:
            return int(s, 16)
        except ValueError:
            try:
                return float(s)
            except ValueError:
                return None


def parse_file(file_name):
    """Parses a file in the form:

    Time, Packet ID, MOSI, MISO\n
    0.0,0,0x00,0x01\n
    0.1,1,0x01,0x02\n
    ...

    All lines that contain the same Packet ID are combined into
    single messages and then sent to the parsing object.

    """
    decoder = Decode()

    with open(file_name) as in_file:
        _verify_column_names(in_file.readline())

        start_ts = None
        cur_packet_id = None
        mosi_data = []
        miso_data = []
        for line in in_file:
            ts, packet_id, mosi, miso = [_parse_num(n) for n in
                                         line.split(COL_SEPARATOR)]

            if (packet_id is None):
                continue

            if (packet_id != cur_packet_id):
                if (cur_packet_id is not None):
                    decoder.update(start_ts,
                                   cur_packet_id,
                                   mosi_data,
                                   miso_data)

                start_ts = ts
                cur_packet_id = packet_id
                mosi_data = [mosi]
                miso_data = [miso]
            else:
                mosi_data.append(mosi)
                miso_data.append(miso)

        if (cur_packet_id is not None):
            decoder.update(start_ts,
                           cur_packet_id,
                           mosi_data,
                           miso_data)

    return decoder


if ("__main__" == __name__):
    """Parses the SPI trace of a Saleae logic analyzer and creates a version of the
    trace that contains human-readable names and/or creates micro-esb init code.

    USAGE:    python nrf24l01p-decode.py -i in.txt -o out.txt -u uesb.txt
    OPTIONS:
        -i    [required]    Specify the path of the input file to use
        -o    [optional]    Specify the path of the human-readable output file to create
        -u    [optional]    Specify the path of the micro-esb init code file to create

    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input_file', dest='input_file_name')
    parser.add_argument('-o', '--output_file', dest='output_file_name')
    parser.add_argument('-u', '--uesb_config_file', dest='uesb_file')
    args = parser.parse_args()
    if (args.input_file_name is None):
        sys.stderr.write('ERROR: No input file specified\r\n')
        sys.exit(-1)

    decoder = parse_file(args.input_file_name)

    if (args.output_file_name is not None):
        with open(args.output_file_name, 'wb') as out_file:
            out_file.write('nRF24L01 SPI Decoder v' + str(VERSION[0]) + os.linesep)
            out_file.write(datetime.datetime.now().strftime('%c') + os.linesep)
            out_file.write("Input file: '" +
                           os.path.basename(args.input_file_name) +
                           "'" + os.linesep)
            out_file.write('-' * 80 + os.linesep)
            out_file.write('{:<25s} {:s}{:s}'.format('Packet format:',
                                                     decoder.get_packet_format(),
                                                     os.linesep))
            out_file.write('{:<25s} {:s}{:s}'.format('Data rate:',
                                                     decoder.get_data_rate(),
                                                     os.linesep))
            out_file.write('{:<25s} {:s}{:s}'.format('CRC width:',
                                                     decoder.get_CRC_mode(),
                                                     os.linesep))
            out_file.write('{:<25s} {:d}{:s}'.format('Address width:',
                                                     decoder.get_address_width(),
                                                     os.linesep))
            out_file.write('{:<25s} {}{:s}'.format('Possible channels:',
                                                   decoder.get_used_channels(),
                                                   os.linesep))
            out_file.write('{:<25s} {:s}{:s}'.format('Output power:',
                                                     decoder.get_output_power(),
                                                     os.linesep))
            out_file.write('{:<25s} {:d}{:s}'.format('Auto retransmit count:',
                                                     decoder.get_auto_retransmit_count(),
                                                     os.linesep))
            out_file.write('{:<25s} {:d}{:s}'.format('Auto retransmit delay:',
                                                     decoder.get_auto_retransmit_delay(),
                                                     os.linesep))
            out_file.write('{:<25s} {:d}{:s}'.format('Packets sent:',
                                                     decoder.get_tx_count(),
                                                     os.linesep))
            out_file.write('{:<25s} {:d}{:s}'.format('Packets received:',
                                                     decoder.get_rx_count(),
                                                     os.linesep))
            out_file.write('-' * 80 + os.linesep)
            out_file.write(decoder.__repr__())

    if (args.uesb_file is not None):
        with open(args.uesb_file, 'wb') as out_file:
            out_file.write(decoder.get_uesb_config())

    sys.exit(0)
