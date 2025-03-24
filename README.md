FDX-B Pet Microchip Reader based on the ch32v003.

![image](https://github.com/user-attachments/assets/5c0fe582-df12-447c-92f1-f59879649ce7)

![image](https://github.com/user-attachments/assets/d0b7835d-4a80-43cd-970a-63fdd52a016f)


# Hardware
The reader coil should be about 5cm in diameter.
Around 50 turns should be enough.

You should aim for an inductance between 400uH to 1mH and a Q of around 8.
Check the Microchip application note in /docs.

The circuit is quite simple, all of the elements are clearly labled in the schematic.

I have included a schematic dedicated for simulating the circuit.

![image](https://github.com/user-attachments/assets/461a4e33-c356-48ee-98d4-df65ce0ebcf4)

The PCB was designed to be easly for me to etch at home.

# Firmware

## Build
```sh
cd firmware
git submodule update --init --recursive --depth=1
make
```

Alternatively you can define a env variable with the path to your local `ch32fun`:
```sh
export CH32V003FUN=~/ch32fun/ch32fun
```

## How does it work
Timer 1 is used to produce a driving signal for the mosfets that includes deadtime.
An external interrupt is generated on each edge of the signal and this is where most of the decoding happens.

The signal is ecoded with Biphase encoding, meaning that 2 short consecutive periods between signal edges represent a 0 and one longer period represents a 1.

The code uses 2 buffers of 128bits (as 4 x 32bit), one is the working buffer and the other contains the last decoded frame.

Detected bits are loaded from the least significant bit to the most significant bit. The current bit index and a bit mask is used to compare the current state of the buffer to the start condition (1 followed by exactly 10 zeros) as well as to check the control bits. If at any point the buffer doesn't match the expected bit pattern, the bit count and the mask are reset.

Once the working buffer is full, we swap buffers.
We can then parse the raw bit pattern into a struct.

A bunch of optimisations have been made to ensure that the code is as small as possible, but still pretty fast.
Most comparison values have been calculated at compile time and all necesery divisions and modulo opperations have been implemented using bit shifts and bitwise ANDs.

The CRC16K is calculated and verified during the parsing of the bit pattern to minimise copies.

I have included a function that can print a `uint64_t` integer with 0 padding without the use of any decimal division/modulus instructions (which would add about 3k to the binary)

With logs off (but still printing an ID):
```
  FLASH:        2592 B        16 KB     15.82%
    RAM:          60 B         2 KB      2.93%
```


# Resources
 - Nice description of the protocol: https://www.priority1design.com.au/fdx-b_animal_identification_protocol.html
 - Similar project base on Arduino: https://github.com/decrazyo/fdxb
 - FDX-B decoder for sigrok: https://github.com/swdee/fdx-b-decoder
