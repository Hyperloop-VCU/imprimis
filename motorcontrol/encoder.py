'''
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

'''




