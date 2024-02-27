#import RPI.GPIO as GPIO


"""
# PINS FOR INPUT
    + 5, 6, 16 (Left Encoder)
        + 5 -> A
        + 6 -> B
        + 16 -> index
    + 24, 25, 26 (Right Encoder)
        + 24 -> A
        + 25 -> B
        + 26 -> index

----------------------------------------------------------------

+ index -> origin point to check when system is restarted
    + dont start reading input until you get high from the index
+ A & B -> pulses in which are compared to figure out direction

+ These inputs are present for both encoders seperately

+ B leads A for clockwise shaft rotation, and A leads B for counterclockwise rotation viewed from the cover side of the encoder.

+ Our encoder is 500 pulses per revolution
"""



class Encoders:

    def __init__(self):
        self.leftPreviousEncoderState = 0x0
        self.rightPreviousEncoderState = 0x0
        self.leftEncoderRunning = False
        self.rightEncoderRunning = False
        
    
        #negative if counterclockwise, positive if clockwise
        self.leftEncoderPulses = 0
        self.rightEncoderPulses = 0
        self.leftEncoderRunning = False
        self.leftClockwise = False
        self.rightClockwise = False

    def setupEncoders(self): 
        #allows the use of the pins in multiple scripts
        GPIO.setmode(GPIO.BCM)

        GPIO.setmode(5, GPIO.IN)
        GPIO.setmode(6, GPIO.IN)
        GPIO.setmode(16, GPIO.IN)
        GPIO.setmode(24, GPIO.IN)
        GPIO.setmode(25, GPIO.IN)
        GPIO.setmode(26, GPIO.IN)


    def updateLeftEncoder(self):
        if(not self.leftEncoderRunning): 
            if(GPIO.input(16)):
                self.leftEncoderRunning = True

        currentEncoderState = 0x0

        if(not self.leftEncoderRunning):   
            return(None)
        
        currentEncoderState = (GPIO.input(6)<<1) | (GPIO.input(5))
        
        # if the previous state had just A // counterclockwise
        if((self.leftPreviousEncoderState == 0x1) & (currentEncoderState == 0x3)):
            self.leftEncoderPulses -= 1
            self.leftClockwise = False
        # if the previous state had just a B // clockwise
        if((self.leftPreviousEncoderState == 0x2) & (currentEncoderState == 0x3)):
            self.leftEncoderPulses += 1
            self.leftClockwise = True

        self.leftPreviousEncoderState = currentEncoderState

    def updateRightEncoder(self): 
        if(not self.rightEncoderRunning):
            if(GPIO.input(26)):
                self.rightEncoderRunning = True

        currentEncoderState = 0x0

        if(not self.rightEncoderRunning):   
            return(None)
        
        currentEncoderState = (GPIO.input(25)<<1) | (GPIO.input(24))
        
        # if the previous state had just A // counterclockwise
        if((self.rightPreviousEncoderState == 0x1) & (currentEncoderState == 0x3)):
            self.rightEncoderPulses -= 1
            self.rightClockwise = False

        # if the previous state had just a B // clockwise
        if((self.rightPreviousEncoderState == 0x2) & (currentEncoderState == 0x3)):
            self.rightEncoderPulses += 1
            self.rightClockwise = True

        self.rightPreviousEncoderState = currentEncoderState




    # left pins: A: 5 B: 6 Index: 16
    # A = first bit 
    # B = second bit 
    def pulseLeftEncoder(self):  
        self.updateLeftEncoder()
        return(self.leftEncoderPulses)

    def pulseRightEncoder(self):
        self.updateRightEncoder()
        return(self.rightEncoderPulses)

                      

if __name__ == "__main__": 
    encoders = Encoders()
    encoders.setupEncoders()
    while(1):
        print(f"[RIGHT] {encoders.pulseRightEncoder=} {encoders.rightClockwise=}")
        print(f"[LEFT] {encoders.pulseLeftEncoder=} {encoders.leftClockwise=}")
        
