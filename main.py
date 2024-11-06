import time
import struct
import spidev
import RPi.GPIO as GPIO

class LoRa:
    def __init__(self, RST_Pin, CS_Pin, SPI_CH, SCK_Pin, MOSI_Pin, MISO_Pin, DIO0_Pin, plus20dBm=False):
        # Set up GPIO
        self.RST_Pin = RST_Pin
        self.CS_Pin = CS_Pin
        self.DIO0_Pin = DIO0_Pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.RST_Pin, GPIO.OUT)
        GPIO.setup(self.CS_Pin, GPIO.OUT)
        GPIO.setup(self.DIO0_Pin, GPIO.IN)

        # Reset LoRa Module
        GPIO.output(self.RST_Pin, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(self.RST_Pin, GPIO.HIGH)
        time.sleep(0.01)

        # Initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_CH, 0)  # SPI channel, SPI device (0 for CE0)
        self.spi.max_speed_hz = 10000000  # 10MHz
        self.spi.mode = 0b00

        # Register table
        self.RegTable = {
            'RegFifo'              : 0x00 ,
            'RegOpMode'            : 0x01 , # operation mode
            'RegFrfMsb'            : 0x06 ,
            'RegFrfMid'            : 0x07 ,
            'RegFrfLsb'            : 0x08 ,
            'RegPaConfig'          : 0x09 ,
            'RegFifoTxBaseAddr'    : 0x0e ,
            'RegFifoRxBaseAddr'    : 0x0f ,
            'RegFifoAddrPtr'       : 0x0d ,
            'RegFifoRxCurrentAddr' : 0x10 ,
            'RegIrqFlags'          : 0x12 ,  
            'RegRxNbBytes'         : 0x13 , # Number of received bytes 
            'RegPktSnrValue'       : 0x19 ,
            'RegPktRssiValue'      : 0x1a ,
            'RegRssiValue'         : 0x1b ,
            'RegModemConfig1'      : 0x1d , 
            'RegModemConfig2'      : 0x1e , 
            'RegPreambleMsb'       : 0x20 , 
            'RegPreambleLsb'       : 0x21 ,
            'RegPayloadLength'     : 0x22 ,
            'RegModemConfig3'      : 0x26 , 
            'RegDioMapping1'       : 0x40 , 
            'RegVersion'           : 0x42 , 
            'RegPaDac'             : 0x4d 
        }

        # Mode settings
        self.Mode = {
            'SLEEP': 0b000,
            'STANDBY': 0b001,
            'TX': 0b011,
            'RXCONTINUOUS': 0b101,
            'RXSINGLE': 0b110,
            'CAD': 0b111,
        }

        # Choose LoRa mode and Test write/read functions
        LongRangeMode = 0b1
        # Choose LoRa (instead of FSK) mode for SX1276 and put the module in sleep mode
        self.write('RegOpMode', self.Mode['SLEEP'] | LongRangeMode << 7) 
        # Test read function 
        assert self.read('RegOpMode') == (self.Mode['SLEEP'] | LongRangeMode << 7), "LoRa initialization failed"
         
        # Set modem config: bandwidth, coding rate, header mode, spreading factor, CRC, and etc.  
        # See 4.4. LoRa Mode Register Map 
        Bw                   = {'125KHz':0b0111, '500kHz':0b1001}
        CodingRate           = {5:0b001, 6:0b010, 7:0b011, 8:0b100}
        ImplicitHeaderModeOn = {'Implicit':0b1, 'Explicit':0b0}
        self.write('RegModemConfig1', Bw['125KHz'] << 4 | CodingRate[8] << 1 | ImplicitHeaderModeOn['Explicit'])
        SpreadingFactor  = {7:0x7, 9:0x9, 12:0xC}
        TxContinuousMode = {'normal':0b0, 'continuous':0b1}
        RxPayloadCrcOn   = {'disable':0b0, 'enable':0b1}
        self.write('RegModemConfig2', SpreadingFactor[12] << 4 | TxContinuousMode['normal'] << 3 | RxPayloadCrcOn['enable'] << 2 | 0x00) 
        LowDataRateOptimize = {'Disabled':0b0, 'Enabled':0b1}
        AgcAutoOn = {'register LnaGain':0b0, 'internal AGC loop':0b1}
        self.write('RegModemConfig3', LowDataRateOptimize['Enabled'] << 3 | AgcAutoOn['internal AGC loop'] << 2)  
        
        # Preamble length
        self.write('RegPreambleMsb', 0x0) # Preamble can be (2^15)kb long, much longer than payload
        self.write('RegPreambleLsb', 0x8) # but we just use 8-byte preamble
        
        # See 4.1.4. Frequency Settings
        FXOSC = 32e6 # Freq of XOSC
        FSTEP = FXOSC / (2**19)
        Frf = int(915e6 / FSTEP)
        self.write('RegFrfMsb', (Frf >> 16) & 0xff)
        self.write('RegFrfMid', (Frf >>  8) & 0xff)
        self.write('RegFrfLsb',  Frf        & 0xff)
        
        # Output Power
        '''
        If desired output power is within -4 ~ +15dBm, use PA_LF or PA_HF as amplifier. 
        Use PA_BOOST as amplifier to output +2 ~ +17dBm continuous power or up to 20dBm 
          peak power in a duty cycled operation.
        Here we will always use PA_BOOST. 
        Since we use PA_BOOST, Pout = 2 + OutputPower and MaxPower could be any number (Why not 0b111/0x7?)
        '''
        PaSelect    = {'PA_BOOST':0b1, 'RFO':0b0} # Choose PA_BOOST (instead of RFO) as the power amplifier
        MaxPower    = {'15dBm':0x7, '13dBm':0x2}  # Pmax = 10.8 + 0.6 * 7  
        OutputPower = {'17dBm':0xf, '2dBm':0x0}  
        self.write('RegPaConfig', PaSelect['PA_BOOST'] << 7 | MaxPower['15dBm'] << 4 | OutputPower['2dBm'])
        
        # Enables the +20dBm option on PA_BOOST pin.  
        if plus20dBm: # PA (Power Amplifier) DAC (Digital Analog Converter)
            PaDac = {'default':0x04, 'enable_PA_BOOST':0x07} # Can be 0x04 or 0x07. 0x07 will enables the +20dBm option on PA_BOOST pin
            self.write('RegPaDac', PaDac['enable_PA_BOOST'])  
        
        # FIFO data buffer 
        '''
        SX1276 has a 256 byte memory area as the FIFO buffer for Tx/Rx operations.
        How do we know which area is for Tx and which is for Rx.
        We must set the base addresses RegFifoTxBaseAddr and RegFifoRxBaseAddr independently.
        Since SX1276 work in a half-duplex manner, we better set both base addresses
        at the bottom (0x00) of the FIFO buffer so that we can buffer 256 byte data
        during transmition or reception.
        ''' 
        self.Fifo_Bottom = 0x00 # We choose this value to max buffer we can write (then send out)
        self.write('RegFifoTxBaseAddr', self.Fifo_Bottom)
        self.write('RegFifoRxBaseAddr', self.Fifo_Bottom)

        self.write('RegOpMode', self.Mode['STANDBY'])

    def write(self, reg, data):
        wb = [self.RegTable[reg] | 0x80]  # Create a writing byte
        if isinstance(data, int):
            data = wb + [data]
        elif isinstance(data, str):
            data = wb + list(data.encode('utf-8'))
        else:
            raise ValueError("Data must be int or str")
        
        GPIO.output(self.CS_Pin, GPIO.LOW)  # Enable communication
        self.spi.xfer(data)
        GPIO.output(self.CS_Pin, GPIO.HIGH)  # Release the bus

    def read(self, reg=None, length=1):
        GPIO.output(self.CS_Pin, GPIO.LOW)
        if length == 1:
            data = self.spi.xfer([self.RegTable[reg], 0x00])
            result = data[1]
        else:
            data = self.spi.xfer([self.RegTable[reg]] + [0x00] * length)
            result = data[1:]
        GPIO.output(self.CS_Pin, GPIO.HIGH)
        return result

    def _irq_handler(self, channel):
        irq_flags = self.read('RegIrqFlags')
        self.write('RegIrqFlags', 0xff)  # Clear all interrupt flags

        if irq_flags & self.IrqFlags['RxDone']:
            if irq_flags & self.IrqFlags['PayloadCrcError']:
                print('PayloadCrcError')
            else:
                self.write('RegFifoAddrPtr', self.read('RegFifoRxCurrentAddr'))
                packet = self.read('RegFifo', self.read('RegRxNbBytes'))
                # Handle received packet
                self.packet_handler(packet)

        elif irq_flags & self.IrqFlags['TxDone']:
            self.after_TxDone()

    def send(self, data):
        self.write('RegFifoAddrPtr', self.Fifo_Bottom)
        self.write('RegFifo', data)
        self.write('RegPayloadLength', len(data))
        self.write('RegOpMode', self.Mode['TX'])

    def packet_handler(self, packet):
        print(f"Received packet: {packet}")

    def after_TxDone(self):
        print('TxDone')

if __name__ == "__main__":
    # Define GPIO pins based on your setup
    LoRa_MISO_Pin = 13
    LoRa_CS_Pin = 6
    LoRa_SCK_Pin = 14
    LoRa_MOSI_Pin = 12
    LoRa_DIO0_Pin = 7
    LoRa_RST_Pin = 0
    SPI_CH = 0

    lora = LoRa(LoRa_RST_Pin, LoRa_CS_Pin, SPI_CH, LoRa_SCK_Pin, LoRa_MOSI_Pin, LoRa_MISO_Pin, LoRa_DIO0_Pin)

    lora.after_TxDone = lambda: print('TxDone')
    
    while True:
        payload = str(random.randint(100, 999)) + ")abc Hello~"
        print(payload)
        lora.send(payload)
        time.sleep(5)
